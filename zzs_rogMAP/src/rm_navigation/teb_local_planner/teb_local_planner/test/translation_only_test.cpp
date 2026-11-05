#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "teb_local_planner/optimal_planner.h"

using namespace teb_local_planner;

TEST(TranslationOnly, VertexKeepsYawButMovesXY)
{
  VertexPose pose(1, 2, .7);
  const double delta[3] = {.2, -.3, .4};
  pose.setYawLocked(true);
  pose.oplusImpl(delta);
  EXPECT_DOUBLE_EQ(pose.x(), 1.2);
  EXPECT_DOUBLE_EQ(pose.y(), 1.7);
  EXPECT_NEAR(pose.theta(), .7, 1e-12);
  pose.setYawLocked(false);
  pose.oplusImpl(delta);
  EXPECT_NEAR(pose.theta(), 1.1, 1e-12);
}

TEST(TranslationOnly, OptimizerWarmStartResizeAndToggle)
{
  if (!rclcpp::ok()) rclcpp::init(0, nullptr);
  auto node = std::make_shared<nav2_util::LifecycleNode>("translation_only_test");
  TebConfig cfg;
  Point2dContainer footprint;
  footprint.emplace_back(.25, .2);
  footprint.emplace_back(.25, -.2);
  footprint.emplace_back(-.25, -.2);
  footprint.emplace_back(-.25, .2);
  cfg.robot_model = std::make_shared<PolygonRobotFootprint>(footprint);
  cfg.trajectory.translation_only = true;
  cfg.trajectory.shortest_rotation_initialization = true; // 被新模式覆盖
  cfg.hcp.enable_homotopy_class_planning = false;
  cfg.robot.max_vel_y = .5;
  cfg.robot.acc_lim_y = .5;
  cfg.optim.weight_kinematics_nh = 0;
  cfg.optim.weight_kinematics_forward_drive = 0;
  ObstContainer obstacles;
  obstacles.push_back(std::make_shared<PointObstacle>(.8, .8));
  TebOptimalPlanner planner(node, cfg, &obstacles);
  std::vector<geometry_msgs::msg::PoseStamped> path(4);
  for (size_t i=0; i<path.size(); ++i) {
    path[i].pose.position.x = double(i)/3;
    path[i].pose.position.y = .5*double(i)/3;
    const double yaw = i == 0 ? .3 : 2.9;
    path[i].pose.orientation.z = std::sin(yaw/2);
    path[i].pose.orientation.w = std::cos(yaw/2);
  }
  geometry_msgs::msg::Twist velocity;
  velocity.angular.z = .6;
  ASSERT_TRUE(planner.plan(path, &velocity, false));
  for (int i=0; i<planner.teb().sizePoses(); ++i) {
    EXPECT_NEAR(planner.teb().Pose(i).theta(), .3, 1e-12);
    EXPECT_TRUE(planner.teb().PoseVertex(i)->yawLocked());
  }
  double vx, vy, omega;
  ASSERT_TRUE(planner.getVelocityCommand(vx, vy, omega, 1));
  EXPECT_TRUE(std::isfinite(vx));
  EXPECT_TRUE(std::isfinite(vy));
  EXPECT_GT(std::hypot(vx, vy), 0);
  EXPECT_NEAR(omega, 0, 1e-12);
  path[0].pose.position.x = .05;
  path[0].pose.orientation.z = std::sin(.35/2);
  path[0].pose.orientation.w = std::cos(.35/2);
  ASSERT_TRUE(planner.plan(path, &velocity, false));
  for (int i=0; i<planner.teb().sizePoses(); ++i)
    EXPECT_NEAR(planner.teb().Pose(i).theta(), .35, 1e-12);
  cfg.trajectory.translation_only = false;
  ASSERT_TRUE(planner.plan(path, &velocity, false));
  EXPECT_NEAR(planner.teb().BackPose().theta(), 2.9, 1e-12);
  for (int i=0; i<planner.teb().sizePoses(); ++i)
    EXPECT_FALSE(planner.teb().PoseVertex(i)->yawLocked());
  cfg.trajectory.translation_only = true;
  cfg.robot.max_vel_y = 0.0;
  EXPECT_FALSE(planner.plan(path, &velocity, false));
  cfg.robot.max_vel_y = .5;
  cfg.hcp.enable_homotopy_class_planning = true;
  EXPECT_FALSE(planner.plan(path, &velocity, false));
  cfg.hcp.enable_homotopy_class_planning = false;
  cfg.trajectory.follow_global_plan_orientation = true;
  EXPECT_FALSE(planner.plan(path, &velocity, false));
}
