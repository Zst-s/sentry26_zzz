/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <tf2/time.h>
#include <tf2_ros/buffer_interface.h>

#include "teb_local_planner/optimal_planner.h"
#include "teb_local_planner/shortest_rotation.h"

#include <map>
#include <exception>
// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_shortest_path.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_via_point.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>

#include <memory>
#include <limits>
#include <sstream>
#include <map>
#include <typeinfo>
#include <algorithm>

namespace teb_local_planner
{

// The edge attracts orientation only; the final tolerance check below makes it a strict constraint.
class EdgeGlobalPlanOrientation : public BaseTebUnaryEdge<1, double, VertexPose>
{
public:
  void computeError() override
  {
    const auto* vertex = static_cast<const VertexPose*>(_vertices[0]);
    _error[0] = g2o::normalize_theta(vertex->pose().theta() - _measurement);
  }
};

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(nullptr), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  initialize(node, cfg, obstacles, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::initialize(nav2_util::LifecycleNode::SharedPtr node, const TebConfig& cfg, ObstContainer* obstacles, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  node_ = node;
  if (!node_->has_parameter("teb_diagnostics.enabled"))
    node_->declare_parameter("teb_diagnostics.enabled", true);
  if (!node_->has_parameter("teb_diagnostics.watch_xy"))
    node_->declare_parameter("teb_diagnostics.watch_xy", std::vector<double>{-0.17, 1.55, -0.15, 1.55});
  if (!node_->has_parameter("teb_diagnostics.watch_radius"))
    node_->declare_parameter("teb_diagnostics.watch_radius", 0.015);
  if (!node_->has_parameter("teb_diagnostics.watch_all"))
    node_->declare_parameter("teb_diagnostics.watch_all", true);
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  // 默认不偏好顺时针或逆时针旋转
  prefer_rotdir_ = RotType::none;

  //初始化轨迹起点速度
  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  // 初始化轨迹终点速度
  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
  initialized_ = true;// 标记 TEB 规划器已完成初始化

  setVisualization(visual);
}

void TebOptimalPlanner::setVisualization(const TebVisualizationPtr& visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;

  visualization_->publishLocalPlanAndPoses(teb_);
  
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *cfg_->robot_model);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
 
}

/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", std::make_shared<g2o::HyperGraphElementCreator<VertexPose>>());
  factory->registerType("VERTEX_TIMEDIFF", std::make_shared<g2o::HyperGraphElementCreator<VertexTimeDiff>>());
  factory->registerType("EDGE_TIME_OPTIMAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeTimeOptimal>>());
  factory->registerType("EDGE_SHORTEST_PATH", std::make_shared<g2o::HyperGraphElementCreator<EdgeShortestPath>>());
  factory->registerType("EDGE_VELOCITY", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocity>>());
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>>());
  factory->registerType("EDGE_ACCELERATION", std::make_shared<g2o::HyperGraphElementCreator<EdgeAcceleration>>());
  factory->registerType("EDGE_ACCELERATION_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationStart>>());
  factory->registerType("EDGE_ACCELERATION_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationGoal>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>>());
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", std::make_shared<g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>>());
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>>());
  factory->registerType("EDGE_KINEMATICS_CARLIKE", std::make_shared<g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>>());
  factory->registerType("EDGE_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeObstacle>>());
  factory->registerType("EDGE_INFLATED_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeInflatedObstacle>>());
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", std::make_shared<g2o::HyperGraphElementCreator<EdgeDynamicObstacle>>());
  factory->registerType("EDGE_VIA_POINT", std::make_shared<g2o::HyperGraphElementCreator<EdgeViaPoint>>());
  factory->registerType("EDGE_PREFER_ROTDIR", std::make_shared<g2o::HyperGraphElementCreator<EdgePreferRotDir>>());
  return;
}
// 类型	作用
// VertexPose	机器人在某个时刻的位姿 \((x,y,\theta)\)
// VertexTimeDiff	相邻轨迹位姿之间的时间间隔
// EdgeTimeOptimal	优化轨迹总耗时
// EdgeShortestPath	约束路径长度，倾向于更短的路径
// EdgeVelocity	非全向机器人的速度约束
// EdgeVelocityHolonomic	全向机器人的速度约束
// EdgeAcceleration	非全向机器人的加速度约束
// EdgeAccelerationStart	起点加速度约束
// EdgeAccelerationGoal	终点加速度约束
// EdgeAccelerationHolonomic	全向机器人加速度约束
// EdgeAccelerationHolonomicStart	全向机器人起点加速度约束
// EdgeAccelerationHolonomicGoal	全向机器人终点加速度约束
// EdgeKinematicsDiffDrive	差速机器人运动学约束
// EdgeKinematicsCarlike	汽车式机器人运动学约束
// EdgeObstacle	障碍物距离约束
// EdgeInflatedObstacle	障碍物膨胀距离相关约束
// EdgeDynamicObstacle	动态障碍物避障约束
// EdgeViaPoint	途经点约束
// EdgePreferRotDir	偏好旋转方向约束

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
std::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static std::once_flag flag;
  std::call_once(flag, [this](){this->registerG2OTypes();});

  // allocating the optimizer
  std::shared_ptr<g2o::SparseOptimizer> optimizer = std::make_shared<g2o::SparseOptimizer>();
  auto linearSolver = std::make_unique<TEBLinearSolver>(); // see typedef in optimization.h
  linearSolver->setBlockOrdering(true);
  auto blockSolver = std::make_unique<TEBBlockSolver>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}


bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  if (cfg_->trajectory.translation_only && teb_.sizePoses() > 0) {
    const double yaw = teb_.Pose(0).theta();
    for (int i=0; i<teb_.sizePoses(); ++i) {
      teb_.Pose(i).theta() = yaw;
      teb_.PoseVertex(i)->setYawLocked(true);
    }
    // 底盘只平移模型的边界角速度为零，不把云台角速度当作底盘角速度。
    vel_start_.second.angular.z = 0.0;
    vel_goal_.second.angular.z = 0.0;
  }
  if (cfg_->optim.optimization_activate==false) 
    return false;
  
  bool success = false;
  optimized_ = false;
  
  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  diagnostic_cycle_ = false;
  if (node_->get_parameter("teb_diagnostics.enabled").as_bool() &&
      std::chrono::steady_clock::now() - diagnostic_last_ >= std::chrono::milliseconds(500))
  {
    diagnostic_last_ = std::chrono::steady_clock::now();
    diagnostic_cycle_ = true;
    ++diagnostic_sequence_;
    publishOptimizationDiagnostics("input", -1);
  }
  
  for(int i=0; i<iterations_outerloop; ++i)
  {
    if (cfg_->trajectory.teb_autosize)
    {
      //teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples);
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);

    }

    success = buildGraph(weight_multiplier);
    if (!success) 
    {
        clearGraph();
        return false;
    }
    if (diagnostic_cycle_) publishOptimizationDiagnostics("before", i);
    success = optimizeGraph(iterations_innerloop, false);
    if (diagnostic_cycle_) publishOptimizationDiagnostics(success ? "after" : "failed", i);
    if (!success) 
    {
        clearGraph();
        return false;
    }
    optimized_ = true;
    
    if (compute_cost_afterwards && i==iterations_outerloop-1) // compute cost vec only in the last iteration
      computeCurrentCost(obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
      
    clearGraph();
    
    weight_multiplier *= cfg_->optim.weight_adapt_factor;
  }

  if (cfg_->trajectory.follow_global_plan_orientation)
  {
    if (orientation_reference_plan_.size() < 2)
    {
      optimized_ = false;
      RCLCPP_ERROR(node_->get_logger(), "Global-plan orientation constraint requires a reference path with at least two poses.");
      return false;
    }
    for (int i = 1; i < teb_.sizePoses() - 1; ++i)
    {
      const double target = globalPlanOrientationAt(teb_.Pose(i));
      const double error = std::abs(g2o::normalize_theta(teb_.Pose(i).theta() - target));
      if (error > cfg_->trajectory.global_plan_orientation_tolerance)
      {
        optimized_ = false;
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
          "Rejecting TEB trajectory: pose %d global-path yaw error %.3f rad exceeds %.3f rad.",
          i, error, cfg_->trajectory.global_plan_orientation_tolerance);
        return false;
      }
    }
  }
  return true;
}

void TebOptimalPlanner::publishOptimizationDiagnostics(const std::string& stage, int iteration)
{
  // 这些话题包含优化候选，不代表已通过碰撞检查，绝不能用于驱动机器人。
  try
  {
    if (!diagnostic_markers_)
    {
      diagnostic_markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "teb_diagnostics/markers", rclcpp::QoS(20).best_effort());
      diagnostic_details_ = node_->create_publisher<std_msgs::msg::String>(
        "teb_diagnostics/details", rclcpp::QoS(20).best_effort());
      diagnostic_markers_->on_activate();
      diagnostic_details_->on_activate();
    }
    visualization_msgs::msg::MarkerArray output;
    visualization_msgs::msg::Marker line;
    line.header.frame_id = cfg_->map_frame;
    line.header.stamp = node_->now();
    std::ostringstream instance;
    instance << this;
    line.ns = instance.str() + "/" + stage + "/" + std::to_string(iteration);
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.015;
    line.color.a = 0.9;
    line.color.r = stage == "before" ? 1.0 : 0.1;
    line.color.g = stage == "after" ? 1.0 : 0.2;
    line.color.b = stage == "input" ? 1.0 : 0.1;
    line.lifetime = rclcpp::Duration::from_seconds(0.6);
    auto point = [](double x, double y) {
      geometry_msgs::msg::Point p; p.x = x; p.y = y; return p;
    };
    std::ostringstream report;
    report.precision(9);
    report << "instance=" << instance.str() << " cycle=" << diagnostic_sequence_
           << " stage=" << stage << " outer=" << iteration
           << " stamp_ns=" << node_->now().nanoseconds() << " frame=" << cfg_->map_frame
           << " initialization=" << diagnostic_initialization_ << "\n";
    report << "translation_only=" << cfg_->trajectory.translation_only << "\n";
    // extracted 指进入优化器障碍容器，并不等于原始点云/地图中不存在该障碍。
    const auto watch_xy = node_->get_parameter("teb_diagnostics.watch_xy").as_double_array();
    const double watch_radius = node_->get_parameter("teb_diagnostics.watch_radius").as_double();
    const bool watch_all = node_->get_parameter("teb_diagnostics.watch_all").as_bool();
    report << "obstacle_input_count=" << (obstacles_ ? obstacles_->size() : 0)
           << " watch_all=" << watch_all << "\n";
    // 全量输入只在 input 阶段打印一次；保留质心和几何类型，供跨周期比较。
    if (watch_all && stage == "input" && obstacles_)
      for (size_t index = 0; index < obstacles_->size(); ++index)
      {
        const auto& obstacle = obstacles_->at(index);
        const auto c = obstacle->getCentroid();
        report << "obstacle_input index=" << index << " centroid=" << c.x() << "," << c.y()
               << " dynamic=" << obstacle->isDynamic() << " type=" << typeid(*obstacle).name() << "\n";
      }
    if (watch_xy.size() % 2 || !std::isfinite(watch_radius) || watch_radius <= 0)
      report << "watch_error invalid_watch_xy_or_radius\n";
    else for (size_t k = 0; k + 1 < watch_xy.size(); k += 2)
    {
      size_t count = 0;
      double nearest = std::numeric_limits<double>::infinity();
      if (obstacles_) for (const auto& obstacle : *obstacles_)
      {
        const auto center = obstacle->getCentroid();
        const double d = std::hypot(center.x() - watch_xy[k], center.y() - watch_xy[k+1]);
        nearest = std::min(nearest, d);
        if (d <= watch_radius) {
          ++count;
          report << "watch_candidate target=" << k/2 << " centroid=" << center.x() << "," << center.y() << "\n";
        }
      }
      report << "watch_input target=" << k/2 << " xy=" << watch_xy[k] << "," << watch_xy[k+1]
             << " radius=" << watch_radius << " extracted=" << (count > 0)
             << " matches=" << count << " nearest_centroid_distance=" << nearest << "\n";
    }
    if (stage != "input") report << diagnostic_obstacle_decisions_;
    for (int i = 0; i < teb_.sizePoses(); ++i)
    {
      const auto& pose = teb_.Pose(i);
      line.points.push_back(point(pose.x(), pose.y()));
      report << "pose " << i << " " << pose.x() << " " << pose.y() << " " << pose.theta();
      if (i < teb_.sizeTimeDiffs()) report << " dt=" << teb_.TimeDiff(i);
      report << "\n";
    }
    output.markers.push_back(line);
    auto associations = line;
    associations.id = 1;
    associations.type = visualization_msgs::msg::Marker::LINE_LIST;
    associations.scale.x = 0.005;
    associations.color.r = 1.0; associations.color.g = 0.7; associations.color.b = 0.0;
    associations.points.clear();
    auto via_links = associations;
    via_links.id = 2;
    via_links.color.r = 0.8; via_links.color.g = 0.0; via_links.color.b = 1.0;
    std::map<std::string, double> costs;
    // 从真正建立的优化边读取关联，避免把候选障碍误当作已参与优化的障碍。
    for (auto* raw_edge : optimizer_->edges())
    {
      auto* edge = dynamic_cast<g2o::OptimizableGraph::Edge*>(raw_edge);
      if (!edge) continue;
      edge->computeError();
      costs[typeid(*edge).name()] += edge->chi2();
      const Obstacle* obstacle = nullptr;
      if (auto* e = dynamic_cast<EdgeObstacle*>(edge)) obstacle = e->measurement();
      if (auto* e = dynamic_cast<EdgeInflatedObstacle*>(edge)) obstacle = e->measurement();
      auto* pose_vertex = dynamic_cast<VertexPose*>(edge->vertex(0));
      if (!pose_vertex) continue;
      int pose_index = -1;
      for (int i = 0; i < teb_.sizePoses(); ++i)
        if (teb_.PoseVertex(i) == pose_vertex) { pose_index = i; break; }
      const auto& pose = pose_vertex->pose();
      if (obstacle)
      {
        const auto center = obstacle->getCentroid();
        associations.points.push_back(point(pose.x(), pose.y()));
        associations.points.push_back(point(center.x(), center.y()));
        report << "obstacle_link pose=" << pose_index << " centroid=" << center.x() << ","
               << center.y() << " footprint_distance="
               << cfg_->robot_model->calculateDistance(pose, obstacle) << "\n";
      }
      if (auto* e = dynamic_cast<EdgeViaPoint*>(edge))
      {
        const auto* vp = e->measurement();
        via_links.points.push_back(point(pose.x(), pose.y()));
        via_links.points.push_back(point(vp->x(), vp->y()));
        report << "via_link pose=" << pose_index << " xy=" << vp->x() << "," << vp->y() << "\n";
      }
    }
    output.markers.push_back(associations);
    output.markers.push_back(via_links);
    auto inputs = line;
    inputs.id = 3;
    inputs.type = visualization_msgs::msg::Marker::POINTS;
    inputs.scale.x = inputs.scale.y = 0.025;
    inputs.color.r = inputs.color.g = inputs.color.b = 1.0;
    inputs.points.clear();
    if (obstacles_) for (const auto& obstacle : *obstacles_)
    {
      const auto center = obstacle->getCentroid();
      inputs.points.push_back(point(center.x(), center.y()));
    }
    output.markers.push_back(inputs);
    if (stage == "input")
      for (const auto& p : diagnostic_reference_plan_)
        report << "reference_pose " << p.pose.position.x << " " << p.pose.position.y
               << " quaternion=" << p.pose.orientation.x << "," << p.pose.orientation.y
               << "," << p.pose.orientation.z << "," << p.pose.orientation.w << "\n";
    if (via_points_) for (const auto& vp : *via_points_)
      report << "via_input " << vp.x() << " " << vp.y() << "\n";
    for (const auto& cost : costs)
      report << "weighted_chi2 " << cost.first << " " << cost.second << "\n";
    std_msgs::msg::String details;
    details.data = report.str();
    diagnostic_markers_->publish(output);
    diagnostic_details_->publish(details);
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(node_->get_logger(), "TEB diagnostic publication failed: %s", ex.what());
  }
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::msg::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::msg::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::msg::PoseStamped>& initial_plan, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
{    
  TEB_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (initial_plan.size() < 2) return false;
  if (cfg_->trajectory.translation_only &&
      (cfg_->robot.max_vel_y <= 0 || cfg_->robot.cmd_angle_instead_rotvel ||
       cfg_->hcp.enable_homotopy_class_planning || cfg_->trajectory.follow_global_plan_orientation)) {
    RCLCPP_ERROR(node_->get_logger(), "translation_only requires holonomic motion, no car steering, no homotopy, and follow_global_plan_orientation=false");
    return false;
  }
  if (previous_translation_only_ != cfg_->trajectory.translation_only)
    teb_.clearTimedElasticBand();
  previous_translation_only_ = cfg_->trajectory.translation_only;
  orientation_reference_plan_ = initial_plan;
  // The controller replaces the first plan pose with the measured robot pose.
  // Its yaw is not the global planner's yaw, so use the next reference yaw there.
  orientation_reference_plan_.front().pose.orientation = initial_plan[1].pose.orientation;
  if (cfg_->trajectory.follow_global_plan_orientation && !previous_follow_global_plan_orientation_)
    teb_.clearTimedElasticBand(); // do not reuse a trajectory optimized without the yaw constraint
  previous_follow_global_plan_orientation_ = cfg_->trajectory.follow_global_plan_orientation;
  const bool shortest_rotation = cfg_->trajectory.shortest_rotation_initialization && !cfg_->trajectory.translation_only;
  if (shortest_rotation &&
      (cfg_->robot.max_vel_y <= 0.0 || cfg_->trajectory.follow_global_plan_orientation))
  {
    RCLCPP_ERROR(node_->get_logger(),
      "shortest_rotation_initialization requires a holonomic robot and follow_global_plan_orientation=false.");
    return false;
  }
  if (shortest_rotation != previous_shortest_rotation_initialization_)
    teb_.clearTimedElasticBand(); // discard the previous mode's angular branch
  previous_shortest_rotation_initialization_ = shortest_rotation;
  diagnostic_initialization_ = shortest_rotation ? "shortest_rotation_fresh" : "fresh";
  // Keep reference poses unchanged in the diagnostics. Only the initialization
  // seed is changed; optimizer constraints and subsequent collision checks remain active.
  auto seed_plan = shortest_rotation ? makeShortestRotationPlan(initial_plan) : initial_plan;
  if (cfg_->trajectory.translation_only) {
    const double yaw = PoseSE2(initial_plan.front().pose).theta();
    for (auto& point : seed_plan) {
      point.pose.orientation.x = point.pose.orientation.y = 0.0;
      point.pose.orientation.z = std::sin(yaw/2);
      point.pose.orientation.w = std::cos(yaw/2);
    }
  }
  const bool estimate_orientation = !cfg_->trajectory.translation_only && !shortest_rotation && cfg_->trajectory.global_plan_overwrite_orientation;
  if (node_->get_parameter("teb_diagnostics.enabled").as_bool())
    diagnostic_reference_plan_ = initial_plan;
  if (!teb_.isInit())
  {
    teb_.initTrajectoryToGoal(seed_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, estimate_orientation,
      cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
  }
  else // warm start
  {
    PoseSE2 start_(seed_plan.front().pose);
    PoseSE2 goal_(seed_plan.back().pose);
    if (teb_.sizePoses()>0
        && (goal_.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal_.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
    {
      diagnostic_initialization_ = "warm_start";
      teb_.updateAndPruneTEB(start_, goal_, cfg_->trajectory.min_samples); // update TEB
    }
    else // goal too far away -> reinit
    {
      RCLCPP_DEBUG(node_->get_logger(), "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      diagnostic_initialization_ = shortest_rotation ? "shortest_rotation_goal_reinit" : "goal_reinit";
      teb_.initTrajectoryToGoal(seed_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta, estimate_orientation,
        cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
  
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
}


//bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
//{
//  PoseSE2 start_(start);
//  PoseSE2 goal_(goal);
//  return plan(start_, goal_, start_vel);
//}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::msg::Twist* start_vel, bool free_goal_vel)
{
  TEB_ASSERT_MSG(initialized_, "Call initialize() first.");
  if (cfg_->trajectory.translation_only) {
    std::vector<geometry_msgs::msg::PoseStamped> plan(2);
    start.toPoseMsg(plan.front().pose);
    goal.toPoseMsg(plan.back().pose);
    return this->plan(plan, start_vel, free_goal_vel);
  }
  if (previous_translation_only_) {
    teb_.clearTimedElasticBand();
    previous_translation_only_ = false;
  }
  orientation_reference_plan_.clear();
  if (cfg_->trajectory.follow_global_plan_orientation)
    return false; // this overload has no global path orientations to follow
  diagnostic_initialization_ = teb_.isInit() ? "pose_api_existing" : "pose_api_fresh";
  diagnostic_reference_plan_.clear();
  if (!teb_.isInit())
  {
    // init trajectory
    teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion); // 0 intermediate samples, but dt=1 -> autoResize will add more samples before calling first optimization
  }
  else // warm start
  {
    if (teb_.sizePoses() > 0
        && (goal.position() - teb_.BackPose().position()).norm() < cfg_->trajectory.force_reinit_new_goal_dist
        && fabs(g2o::normalize_theta(goal.theta() - teb_.BackPose().theta())) < cfg_->trajectory.force_reinit_new_goal_angular) // actual warm start!
      teb_.updateAndPruneTEB(start, goal, cfg_->trajectory.min_samples);
    else // goal too far away -> reinit
    {
      RCLCPP_DEBUG(node_->get_logger(), "New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      teb_.clearTimedElasticBand();
      teb_.initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);
    }
  }
  if (start_vel)
    setVelocityStart(*start_vel);
  if (free_goal_vel)
    setVelocityGoalFree();
  else
    vel_goal_.first = true; // we just reactivate and use the previously set velocity (should be zero if nothing was modified)
      
  // now optimize
  return optimizeTEB(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations, true);
}


bool TebOptimalPlanner::buildGraph(double weight_multiplier)
{
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    RCLCPP_WARN(node_->get_logger(), "Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);
  // autoResize 新增的顶点也必须锁住 yaw。关闭模式时恢复正常更新。
  for (int i=0; i<teb_.sizePoses(); ++i) {
    teb_.PoseVertex(i)->setYawLocked(cfg_->trajectory.translation_only);
    if (cfg_->trajectory.translation_only) teb_.Pose(i).theta() = teb_.Pose(0).theta();
  }
  
  // add TEB vertices
  AddTEBVertices();
  
  // add Edges (local cost functions)
  if (cfg_->obstacles.legacy_obstacle_association)
    AddEdgesObstaclesLegacy(weight_multiplier);
  else
    AddEdgesObstacles(weight_multiplier);

  if (cfg_->obstacles.include_dynamic_obstacles)
    AddEdgesDynamicObstacles();
  
  AddEdgesViaPoints();

  AddEdgesGlobalPlanOrientation();
  
  AddEdgesVelocity();
  
  AddEdgesAcceleration();

  AddEdgesTimeOptimal();	

  AddEdgesShortestPath();
  
  if (!cfg_->trajectory.translation_only) {
  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  AddEdgesPreferRotDir();
  }

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0)
    AddEdgesVelocityObstacleRatio();
    
  return true;  
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  
  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    RCLCPP_WARN(node_->get_logger(), "optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();

  int iter = optimizer_->optimize(no_iterations);

  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
    RCLCPP_ERROR(node_->get_logger(), "optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
    
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for(auto& v : vertices)
      v.second->edges().clear();

    optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  RCLCPP_DEBUG_EXPRESSION(node_->get_logger(), cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}


void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  diagnostic_obstacle_decisions_.clear();
  std::ostringstream decisions;
  decisions.precision(9);
  const auto watched = diagnostic_cycle_ ? node_->get_parameter("teb_diagnostics.watch_xy").as_double_array() : std::vector<double>{};
  const double radius = diagnostic_cycle_ ? node_->get_parameter("teb_diagnostics.watch_radius").as_double() : 0.0;
  const bool watch_all = diagnostic_cycle_ && node_->get_parameter("teb_diagnostics.watch_all").as_bool();
  auto is_watched = [&](const ObstaclePtr& obstacle) {
    if (watch_all) return true;
    const auto c = obstacle->getCentroid();
    for (size_t k = 0; k + 1 < watched.size(); k += 2)
      if (std::hypot(c.x()-watched[k], c.y()-watched[k+1]) <= radius) return true;
    return false;
  };
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
  {
    if (diagnostic_cycle_) diagnostic_obstacle_decisions_ = "watch_build skipped=zero_weight_or_null_container\n";
    return;
  }
  if (diagnostic_cycle_) decisions << "watch_build distance_basis=footprint force_distance="
      << cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor
      << " cutoff_distance=" << cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor
      << " first_last_pose_edges=skipped decisions_at=graph_build\n";
    
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
  
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  auto create_edge = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle) {
    if (inflated)
    {
      EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information_inflated);
      dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
      EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
      dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
      dist_bandpt_obst->setInformation(information);
      dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obstacle);
      optimizer_->addEdge(dist_bandpt_obst);
    };
  };
    
  // iterate all teb points, skipping the last and, if the EdgeVelocityObstacleRatio edges should not be created, the first one too
  const int first_vertex = cfg_->optim.weight_velocity_obstacle_ratio == 0 ? 1 : 0;
  for (int i = first_vertex; i < teb_.sizePoses() - 1; ++i)
  {    
      double left_min_dist = std::numeric_limits<double>::max();
      double right_min_dist = std::numeric_limits<double>::max();
      ObstaclePtr left_obstacle;
      ObstaclePtr right_obstacle;
      
      const Eigen::Vector2d pose_orient = teb_.Pose(i).orientationUnitVec();
      
      // iterate obstacles
      for (const ObstaclePtr& obst : *obstacles_)
      {
        // we handle dynamic obstacles differently below
        if(cfg_->obstacles.include_dynamic_obstacles && obst->isDynamic())
          continue;

          // calculate distance to robot model
          double dist = cfg_->robot_model->calculateDistance(teb_.Pose(i), obst.get());
          
          // force considering obstacle if really close to the current pose
        if (dist < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor)
          {
              iter_obstacle->push_back(obst);
              continue;
          }
          // cut-off distance
          if (dist > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor)
            continue;
          
          // determine side (left or right) and assign obstacle if closer than the previous one
          if (cross2d(pose_orient, obst->getCentroid()) > 0) // left
          {
              if (dist < left_min_dist)
              {
                  left_min_dist = dist;
                  left_obstacle = obst;
              }
          }
          else
          {
              if (dist < right_min_dist)
              {
                  right_min_dist = dist;
                  right_obstacle = obst;
              }
          }
      }   
      
      if (left_obstacle)
        iter_obstacle->push_back(left_obstacle);
      if (right_obstacle)
        iter_obstacle->push_back(right_obstacle);

      if (diagnostic_cycle_) for (size_t obstacle_index = 0; obstacle_index < obstacles_->size(); ++obstacle_index)
      {
        const auto& obstacle = obstacles_->at(obstacle_index);
        if (!is_watched(obstacle)) continue;
        const auto c = obstacle->getCentroid();
        const double d = cfg_->robot_model->calculateDistance(teb_.Pose(i), obstacle.get());
        const bool selected = std::find(iter_obstacle->begin(), iter_obstacle->end(), obstacle) != iter_obstacle->end();
        const char* reason = "not_nearest_on_side";
        if (cfg_->obstacles.include_dynamic_obstacles && obstacle->isDynamic()) reason = "handled_by_dynamic_edges";
        else if (d < cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_force_inclusion_factor) reason = "force_inclusion";
        else if (d > cfg_->obstacles.min_obstacle_dist*cfg_->obstacles.obstacle_association_cutoff_factor) reason = "beyond_cutoff";
        else if (selected) reason = "nearest_on_side";
        if (i == 0) reason = "first_pose_edge_skipped";
        decisions << "watch_association pose=" << i << " obstacle_index=" << obstacle_index
                  << " centroid=" << c.x() << "," << c.y()
                  << " footprint_distance_at_build=" << d << " edge_added=" << (selected && i != 0)
                  << " reason=" << reason << "\n";
      }

      // continue here to ignore obstacles for the first pose, but use them later to create the EdgeVelocityObstacleRatio edges
      if (i == 0)
      {
        ++iter_obstacle;
        continue;
      }

      // create obstacle edges
      for (const ObstaclePtr obst : *iter_obstacle)
        create_edge(i, obst.get());
      ++iter_obstacle;
  }
  if (diagnostic_cycle_) diagnostic_obstacle_decisions_ = decisions.str();
}


void TebOptimalPlanner::AddEdgesObstaclesLegacy(double weight_multiplier)
{
  diagnostic_obstacle_decisions_ = diagnostic_cycle_ ? "watch_build legacy_mode=1 per_pose_reasons_not_available_use_obstacle_link\n" : "";
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information; 
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
    
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;
  information_inflated(1,1) = cfg_->optim.weight_inflation;
  information_inflated(0,1) = information_inflated(1,0) = 0;
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;
    
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (cfg_->obstacles.include_dynamic_obstacles && (*obst)->isDynamic()) // we handle dynamic obstacles differently below
      continue; 
    
    int index;
    
    if (cfg_->obstacles.obstacle_poses_affected >= teb_.sizePoses())
      index =  teb_.sizePoses() / 2;
    else
      index = teb_.findClosestTrajectoryPose(*(obst->get()));
     
    
    // check if obstacle is outside index-range between start and goal
    if ( (index <= 1) || (index > teb_.sizePoses()-2) ) // start and goal are fixed and findNearestBandpoint finds first or last conf if intersection point is outside the range
	    continue; 
        
    if (inflated)
    {
        EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information_inflated);
        dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }
    else
    {
        EdgeObstacle* dist_bandpt_obst = new EdgeObstacle;
        dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
        dist_bandpt_obst->setInformation(information);
        dist_bandpt_obst->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
        optimizer_->addEdge(dist_bandpt_obst);
    }

    for (int neighbourIdx=0; neighbourIdx < floor(cfg_->obstacles.obstacle_poses_affected/2); neighbourIdx++)
    {
      if (index+neighbourIdx < teb_.sizePoses())
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_r = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information_inflated);
                dist_bandpt_obst_n_r->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_r = new EdgeObstacle;
                dist_bandpt_obst_n_r->setVertex(0,teb_.PoseVertex(index+neighbourIdx));
                dist_bandpt_obst_n_r->setInformation(information);
                dist_bandpt_obst_n_r->setParameters(*cfg_, robot_model_.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_r);
            }
      }
      if ( index - neighbourIdx >= 0) // needs to be casted to int to allow negative values
      {
            if (inflated)
            {
                EdgeInflatedObstacle* dist_bandpt_obst_n_l = new EdgeInflatedObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information_inflated);
                dist_bandpt_obst_n_l->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
            else
            {
                EdgeObstacle* dist_bandpt_obst_n_l = new EdgeObstacle;
                dist_bandpt_obst_n_l->setVertex(0,teb_.PoseVertex(index-neighbourIdx));
                dist_bandpt_obst_n_l->setInformation(information);
                dist_bandpt_obst_n_l->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
                optimizer_->addEdge(dist_bandpt_obst_n_l);
            }
      }
    } 
    
  }
}


void TebOptimalPlanner::AddEdgesDynamicObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;
  
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      dynobst_edge->setInformation(information);
      dynobst_edge->setParameters(*cfg_, cfg_->robot_model.get(), obst->get());
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesViaPoints()
{
  if (cfg_->optim.weight_viapoint==0 || via_points_==NULL || via_points_->empty() )
    return; // if weight equals zero skip adding edges!

  int start_pose_idx = 0;
  
  int n = teb_.sizePoses();
  if (n<3) // we do not have any degrees of freedom for reaching via-points
    return;
  
  for (ViaPointContainer::const_iterator vp_it = via_points_->begin(); vp_it != via_points_->end(); ++vp_it)
  {
    
    int index = teb_.findClosestTrajectoryPose(*vp_it, NULL, start_pose_idx);
    if (cfg_->trajectory.via_points_ordered)
      start_pose_idx = index+2; // skip a point to have a DOF inbetween for further via-points
     
    // check if point conicides with goal or is located behind it
    if ( index > n-2 ) 
      index = n-2; // set to a pose before the goal, since we can move it away!
    // check if point coincides with start or is located before it
    if ( index < 1)
    {
      if (cfg_->trajectory.via_points_ordered)
      {
        index = 1; // try to connect the via point with the second (and non-fixed) pose. It is likely that autoresize adds new poses inbetween later.
      }
      else
      {
        RCLCPP_DEBUG(node_->get_logger(), "TebOptimalPlanner::AddEdgesViaPoints(): skipping a via-point that is close or behind the current robot pose.");
        continue; // skip via points really close or behind the current robot pose
      }
    }
    Eigen::Matrix<double,1,1> information;
    information.fill(cfg_->optim.weight_viapoint);
    
    EdgeViaPoint* edge_viapoint = new EdgeViaPoint;
    edge_viapoint->setVertex(0,teb_.PoseVertex(index));
    edge_viapoint->setInformation(information);
    edge_viapoint->setParameters(*cfg_, &(*vp_it));
    optimizer_->addEdge(edge_viapoint);   
  }
}

double TebOptimalPlanner::globalPlanOrientationAt(const PoseSE2& pose) const
{
  const auto& plan = orientation_reference_plan_;
  double best_distance_sq = std::numeric_limits<double>::infinity();
  double best_yaw = tf2::getYaw(plan.front().pose.orientation);
  const Eigen::Vector2d query = pose.position();
  for (size_t i = 1; i < plan.size(); ++i)
  {
    const Eigen::Vector2d a(plan[i - 1].pose.position.x, plan[i - 1].pose.position.y);
    const Eigen::Vector2d b(plan[i].pose.position.x, plan[i].pose.position.y);
    const Eigen::Vector2d segment = b - a;
    const double length_sq = segment.squaredNorm();
    const double fraction = length_sq > 1e-12
      ? std::clamp((query - a).dot(segment) / length_sq, 0.0, 1.0) : 0.0;
    const double distance_sq = (query - a - fraction * segment).squaredNorm();
    if (distance_sq < best_distance_sq)
    {
      best_distance_sq = distance_sq;
      const double yaw_a = tf2::getYaw(plan[i - 1].pose.orientation);
      const double yaw_b = tf2::getYaw(plan[i].pose.orientation);
      best_yaw = g2o::normalize_theta(yaw_a + fraction * g2o::normalize_theta(yaw_b - yaw_a));
    }
  }
  return best_yaw;
}

void TebOptimalPlanner::AddEdgesGlobalPlanOrientation()
{
  if (!cfg_->trajectory.follow_global_plan_orientation || orientation_reference_plan_.size() < 2)
    return;
  Eigen::Matrix<double, 1, 1> information;
  information[0] = 1000.0;
  for (int i = 1; i < teb_.sizePoses() - 1; ++i)
  {
    auto* edge = new EdgeGlobalPlanOrientation;
    edge->setVertex(0, teb_.PoseVertex(i));
    edge->setMeasurement(globalPlanOrientationAt(teb_.Pose(i)));
    edge->setInformation(information);
    optimizer_->addEdge(edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    } 
    
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();  
    
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0) 
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}



void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}


void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    RCLCPP_WARN(node_->get_logger(), "TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio()
{
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1,1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0,1) = information(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.sizePoses() - 1; ++index)
  {
    for (const ObstaclePtr obstacle : (*iter_obstacle++))
    {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0,teb_.PoseVertex(index));
      edge->setVertex(1,teb_.PoseVertex(index + 1));
      edge->setVertex(2,teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(*cfg_, cfg_->robot_model.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

bool TebOptimalPlanner::hasDiverged() const
{
  // Early returns if divergence detection is not active
  if (!cfg_->recovery.divergence_detection_enable)
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
    return false;

  // Grab the statistics of the final iteration
  const auto last_iter_stats = stats_vector.back();

  return last_iter_stats.chi2 > cfg_->recovery.divergence_detection_max_chi_squared;
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph();	
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  
  optimizer_->computeInitialGuess();
  
  cost_ = 0;

  if (alternative_time_cost)
  {
    cost_ += teb_.getSumOfAllTimeDiffs();
    // TEST we use SumOfAllTimeDiffs() here, because edge cost depends on number of samples, which is not always the same for similar TEBs,
    // since we are using an AutoResize Function with hysteresis.
  }
  
  // now we need pointers to all edges -> calculate error for each edge-type
  // since we aren't storing edge pointers, we need to check every edge
  for (std::vector<g2o::OptimizableGraph::Edge*>::const_iterator it = optimizer_->activeEdges().begin(); it!= optimizer_->activeEdges().end(); it++)
  {
    double cur_cost = (*it)->chi2();

    if (dynamic_cast<EdgeObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeInflatedObstacle*>(*it) != nullptr
        || dynamic_cast<EdgeDynamicObstacle*>(*it) != nullptr)
    {
      cur_cost *= obst_cost_scale;
    }
    else if (dynamic_cast<EdgeViaPoint*>(*it) != nullptr)
    {
      cur_cost *= viapoint_cost_scale;
    }
    else if (dynamic_cast<EdgeTimeOptimal*>(*it) != nullptr && alternative_time_cost)
    {
      continue; // skip these edges if alternative_time_cost is active
    }
    cost_ += cur_cost;
  }

  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}


void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }
  
  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  
  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;    
  }
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    RCLCPP_ERROR(node_->get_logger(), "TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }
  if (dt<=0)
  {	
    RCLCPP_ERROR(node_->get_logger(), "TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::msg::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity 
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }
  
  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<teb_msgs::msg::TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();
  
  trajectory.resize(n);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  teb_msgs::msg::TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start = durationFromSec(curr_time);
  
  curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    teb_msgs::msg::TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);    
    point.time_from_start = durationFromSec(curr_time);
    
    curr_time += teb_.TimeDiff(i);
  }
  
  // goal
  teb_msgs::msg::TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start = durationFromSec(curr_time);
}


bool TebOptimalPlanner::isTrajectoryFeasible(dwb_critics::ObstacleFootprintCritic* costmap_model, const std::vector<geometry_msgs::msg::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx, double feasibility_check_lookahead_distance)
{
  // Steady clock keeps diagnostic throttling independent of ROS simulation time.
  static rclcpp::Clock diagnostic_clock(RCL_STEADY_TIME);
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;

  if (feasibility_check_lookahead_distance > 0){
    for (int i=1; i < teb().sizePoses(); ++i){
      double pose_distance=std::hypot(teb().Pose(i).x()-teb().Pose(0).x(), teb().Pose(i).y()-teb().Pose(0).y());
      if(pose_distance > feasibility_check_lookahead_distance){
        look_ahead_idx = i - 1;
        break;
      }
    }
  }

  geometry_msgs::msg::Pose2D pose2d;
  for (int i=0; i <= look_ahead_idx; ++i)
  {
    teb().Pose(i).toPoseMsg(pose2d);
    if (!isPoseValid(pose2d, costmap_model, footprint_spec)){
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_feasibility"), diagnostic_clock, 1000,
          "[TEB_FEASIBILITY] sample=%d/%d checked_last=%d frame=%s "
          "x=%.4f y=%.4f yaw=%.4f footprint_vertices=%zu inscribed=%.4f circumscribed=%.4f",
          i, static_cast<int>(teb().sizePoses()), look_ahead_idx, cfg_->map_frame.c_str(),
          pose2d.x, pose2d.y, pose2d.theta, footprint_spec.size(), inscribed_radius, circumscribed_radius);
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *cfg_->robot_model);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));
          intermediate_pose.toPoseMsg(pose2d);

          if (!isPoseValid(pose2d, costmap_model, footprint_spec)){
            RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_feasibility"), diagnostic_clock, 1000,
                "[TEB_FEASIBILITY] segment=%d->%d interpolated=%d/%d frame=%s "
                "x=%.4f y=%.4f yaw=%.4f footprint_vertices=%zu inscribed=%.4f circumscribed=%.4f",
                i, i + 1, step + 1, n_additional_samples, cfg_->map_frame.c_str(),
                pose2d.x, pose2d.y, pose2d.theta, footprint_spec.size(), inscribed_radius, circumscribed_radius);
            if (visualization_)
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *cfg_->robot_model);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

bool TebOptimalPlanner::isPoseValid(geometry_msgs::msg::Pose2D pose2d, dwb_critics::ObstacleFootprintCritic* costmap_model,
                           const std::vector<geometry_msgs::msg::Point>& footprint_spec)
{
  static rclcpp::Clock diagnostic_clock(RCL_STEADY_TIME);
  try {
    const double score = costmap_model->scorePose(
        pose2d, dwb_critics::getOrientedFootprint(pose2d, footprint_spec));
    if (score < 0) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_feasibility"), diagnostic_clock, 1000,
          "[TEB_FEASIBILITY] negative_score=%.4f frame=%s x=%.4f y=%.4f yaw=%.4f",
          score, cfg_->map_frame.c_str(), pose2d.x, pose2d.y, pose2d.theta);
      return false;
    }
  } catch (const std::exception &ex) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_feasibility"), diagnostic_clock, 1000,
        "[TEB_FEASIBILITY] critic_exception='%s' frame=%s x=%.4f y=%.4f yaw=%.4f",
        ex.what(), cfg_->map_frame.c_str(), pose2d.x, pose2d.y, pose2d.theta);
    return false;
  } catch (...) {
    RCLCPP_WARN_THROTTLE(rclcpp::get_logger("teb_feasibility"), diagnostic_clock, 1000,
        "[TEB_FEASIBILITY] unknown_exception frame=%s x=%.4f y=%.4f yaw=%.4f",
        cfg_->map_frame.c_str(), pose2d.x, pose2d.y, pose2d.theta);
    return false;
  }
  return true;
}

} // namespace teb_local_planner
