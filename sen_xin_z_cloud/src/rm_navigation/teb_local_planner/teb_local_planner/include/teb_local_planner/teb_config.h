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

#ifndef TEB_CONFIG_H_
#define TEB_CONFIG_H_

#include <nav2_util/lifecycle_node.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <nav_2d_utils/parameters.hpp>
#include "teb_local_planner/robot_footprint_model.h"
#include <nav2_costmap_2d/footprint.hpp>

// Definitions
#define USE_ANALYTIC_JACOBI // if available for a specific edge, use analytic jacobi

namespace teb_local_planner
{
/**
 * @class TebConfig
 * @brief Config class for the teb_local_planner and its components.
 */
class TebConfig
{
public:
  using UniquePtr = std::unique_ptr<TebConfig>;
  
  std::string odom_topic; //!< Topic name of the odometry message, provided by the robot driver or simulator
  std::string map_frame; //!< Global planning frame
  std::string node_name; //!< node name used for parameter event callback

  RobotFootprintModelPtr robot_model;
  std::string model_name;
  double radius;
  std::vector<double> line_start, line_end;
  double front_offset, front_radius, rear_offset, rear_radius;
  std::string footprint_string;

  //! Trajectory related parameters
  struct Trajectory
  {
    double teb_autosize; //!< 启用根据时间分辨率自动调整轨迹大小（推荐）
    double dt_ref; //!< 期望的轨迹时间分辨率（应与底层控制频率处于同一数量级）
    double dt_hysteresis; //!< 依赖于当前时间分辨率（dt）的自动调整滞回值：通常为 dt_ref 的 10%
    double min_segment_time, max_segment_time, max_trajectory_time, weight_time_limit;
    double jerk_limit_x, jerk_limit_y, jerk_limit_theta, weight_jerk;
    int min_samples; //!< 最小采样点数（应始终大于 2）
    int max_samples; //!< 最大采样点数；警告：如果数值太小，离散化/分辨率对于给定的机器人模型可能不够，或者避障功能将失效。
    bool global_plan_overwrite_orientation; //!< 覆盖全局规划器提供的局部子目标点的方向（朝向）
    bool follow_global_plan_orientation; //!< 将全局路径朝向约束到每个中间轨迹点，并拒绝超过容差的结果
    bool shortest_rotation_initialization; //!< 全向车按起终点最短转角初始化中间朝向；不能与严格跟随全局朝向同时启用
    bool translation_only; //!< 全向底盘只平移：每轮以实测起点 yaw 固定全轨迹朝向
    bool goal_approach_enabled; //!< 输出端按最终全局目标距离限制合成平移速度；不改变到达判定
    double goal_approach_distance; //!< 开始减速的欧氏距离 [m]，必须大于零
    double goal_approach_max_speed; //!< 公式中的最大合成平移速度 [m/s]，必须大于零
    double global_plan_orientation_tolerance; //!< 中间轨迹点相对全局路径朝向的最大允许误差 [rad]
    bool allow_init_with_backwards_motion; //!< 如果为 true，当目标点在局部代价地图中位于起点的后方时，底层轨迹可能会以倒车运动进行初始化（仅建议在机器人配备后置传感器时使用）
    double global_plan_viapoint_sep; //!< 从全局规划中提取的每两个连续途经点之间的最小间距（如果为负值：禁用）
    bool via_points_ordered; //!< 如果为 true，规划器将严格遵循存储容器中途经点的顺序
    double max_global_plan_lookahead_dist; //!< 指定用于优化的全局规划子集的最大长度（累积欧氏距离）[如果 <=0：禁用；该长度也受限于局部代价地图的大小！]
    double global_plan_prune_distance; //!< 机器人与全局规划途经点之间用于修剪（裁剪）的距离
    bool exact_arc_length; //!< 如果为 true，规划器在计算速度、加速度和转向角速度时使用精确的弧长 [-> 会增加 CPU 耗时]，否则使用欧氏距离近似。
    double force_reinit_new_goal_dist; //!< 如果更新的目标位置与之前目标的距离超过指定值（以米为单位），则重新初始化轨迹（跳过热启动）
    double force_reinit_new_goal_angular; //!< 如果更新的目标角度与之前目标的差值超过指定值（以弧度为单位），则重新初始化轨迹（跳过热启动）
    int feasibility_check_no_poses; //!< 指定在每个采样间隔内，对预测规划进行可行性检查的位姿数量上限（前提是在 feasibility_check_lookahead_distance 范围内）；如果为 -1，则检查所有位于 feasibility_check_lookahead_distance 范围内的位姿。
    double feasibility_check_lookahead_distance; //!< 指定在每个采样间隔内，从机器人向前多远（且索引低于 feasibility_check_no_poses）进行可行性检查；如果为 -1，则检查直到 feasibility_check_no_poses 的所有位姿。
    bool publish_feedback; //!< 发布包含完整轨迹和活动障碍物列表的规划器反馈（应仅在评估或调试时启用）
    double min_resolution_collision_check_angular; //!< 在代价地图碰撞检查期间使用的最小角度分辨率。如果不满足此分辨率，将添加中间采样点。[弧度]
    int control_look_ahead_poses; //!< 用于提取速度控制指令的位姿索引（前视位姿索引）
  } trajectory; //!< Trajectory related parameters

  //! Robot related parameters
  struct Robot
  {
    double base_max_vel_x; //!< 机器人应用速度限制前的最大平移（直线）速度
    double base_max_vel_x_backwards; //!< 机器人应用速度限制前倒车行驶的最大平移速度
    double base_max_vel_y; //!< 机器人应用速度限制前的最大横移（侧向）速度（对于非全向机器人应设置为零！）
    double base_max_vel_theta; //!< 机器人应用速度限制前的最大角速度
    double max_vel_x; //!< 机器人的最大平移速度
    double max_vel_x_backwards; //!< 机器人倒车行驶的最大平移速度
    double max_vel_y; //!< 机器人的最大横移速度（对于非全向机器人应设置为零！）
    double max_vel_theta; //!< 机器人的最大角速度
    double acc_lim_x; //!< 机器人的最大平移加速度
    double acc_lim_y; //!< 机器人的最大横移加速度
    double acc_lim_theta; //!< 机器人的最大角加速度
    double min_turning_radius; //!< 类车机器人（阿克曼转向）的最小转弯半径（差速驱动机器人设为：零）；
    double wheelbase; //!< 驱动轴与转向轴之间的距离（仅当启用了 'cmd_angle_instead_rotvel' 的类车机器人需要）；对于后轮驱动机器人，该值可能为负数！
    bool cmd_angle_instead_rotvel; //!< 将速度指令消息中的旋转角速度替换为相应的转向角（请检查 'axles_distance' 参数）
    bool is_footprint_dynamic; //<! 如果为 true，则在检查轨迹可行性之前更新机器人的足迹（footprint）
    bool use_proportional_saturation; //<! 如果为 true，当任何速度分量（线速度 x 和 y，以及角速度 z）超出其相应边界时，按比例缩小所有 twist（速度）分量，而不是单独对每个分量进行饱和（截断）处理
    double transform_tolerance = 0.5; //<! 在 TF 树中查询坐标变换时的容差（单位：秒）
  } robot; //!< Robot related parameters

//! 目标容差相关参数
  struct GoalTolerance
  {
    double xy_goal_tolerance; //!< 允许的到达目标位置的最终欧氏距离（即 XY 平面上的位置误差容差）
    bool free_goal_vel; //!< 允许机器人的速度不为零（通常是最大速度 max_vel）以用于规划目的（即到达目标点时不需要强制减速至0）
  } goal_tolerance; //!< 目标容差相关参数

  //! 障碍物相关参数
  struct Obstacles
  {
    double min_obstacle_dist; //!< 与障碍物之间的最小期望间距
    double inflation_dist; //!< 障碍物周围具有非零惩罚代价的缓冲区（为了生效，应该大于 min_obstacle_dist）
    double dynamic_obstacle_inflation_dist; //!< 动态障碍物预测位置周围具有非零惩罚代价的缓冲区（为了生效，应该大于 min_obstacle_dist）
    bool include_dynamic_obstacles; //!< 指定是否应通过恒定速度模型预测动态障碍物的运动（这也会影响同伦类规划）；如果为 false，则所有障碍物均被视为静态。
    bool include_costmap_obstacles; //!< 指定是否应直接考虑代价地图（costmap）中的障碍物
    double costmap_obstacles_behind_robot_dist; //!< 限制在机器人后方规划时考虑的局部代价地图被占据障碍物的距离（以米为单位指定）
    int obstacle_poses_affected; //!< 障碍物位置与轨迹上最近的位姿相关联以减少计算量，但也会考虑一定数量的相邻位姿
    bool legacy_obstacle_association; //!< 如果为 true，则使用旧的关联策略（对于每个障碍物，寻找最近的 TEB 位姿），否则使用新的策略（对于每个 TEB 位姿，仅寻找“相关”的障碍物）。
    double obstacle_association_force_inclusion_factor; //!< 非旧版障碍物关联技术在优化期间仅尝试将相关的障碍物与离散轨迹相连，在指定距离内的所有障碍物都被强制包含在内（作为 min_obstacle_dist 的倍数），例如选择 2.0 以便考虑半径为 2.0*min_obstacle_dist 内的障碍物。
    double obstacle_association_cutoff_factor; //!< 参见 obstacle_association_force_inclusion_factor，但超过 [值]*min_obstacle_dist 的倍数后，优化期间将忽略所有障碍物。首先处理 obstacle_association_force_inclusion_factor。
    std::string costmap_converter_plugin; //!< 定义 costmap_converter 包的一个插件名称（将代价地图栅格单元转换为点/线/多边形）
    bool costmap_converter_spin_thread; //!< 如果为 \c true，代价地图转换器将在不同的线程中调用其回调队列
    int costmap_converter_rate; //!< 定义 costmap_converter 插件处理当前代价地图频率的速率（该值不应远高于代价地图的更新频率）
    double obstacle_proximity_ratio_max_vel; //!< 由于接近静态障碍物而降低速度时，用作上限的最大速度比例
    double obstacle_proximity_lower_bound; //!< 与静态障碍物的距离，当小于该距离时，速度应降低
    double obstacle_proximity_upper_bound; //!< 与静态障碍物的距离，当大于该距离时，速度应提高
  } obstacles; //!< 障碍物相关参数


 //! 优化相关参数
  struct Optimization
  {
    int no_inner_iterations; //!< 每次外循环迭代中调用的求解器（内循环）迭代次数
    int no_outer_iterations; //!< 每次外循环迭代会自动调整轨迹大小，并使用 no_inner_iterations 调用内部优化器

    bool optimization_activate; //!< 激活优化
    bool optimization_verbose; //!< 打印详细（冗长）信息

    double penalty_epsilon; //!< 为硬约束近似的惩罚函数添加一个小的安全裕度

    double weight_max_vel_x; //!< 满足最大允许平移（直线）速度的优化权重
    double weight_max_vel_y; //!< 满足最大允许横移速度的优化权重（仅在全向机器人上使用）
    double weight_max_vel_theta; //!< 满足最大允许角速度的优化权重
    double weight_acc_lim_x; //!< 满足最大允许平移加速度的优化权重
    double weight_acc_lim_y; //!< 满足最大允许横移加速度的优化权重（仅在全向机器人上使用）
    double weight_acc_lim_theta; //!< 满足最大允许角加速度的优化权重
    double weight_kinematics_nh; //!< 满足非全向（非完整约束）运动学的优化权重
    double weight_kinematics_forward_drive; //!< 强制机器人仅选择向前方向（正平移速度，仅限差速驱动机器人）的优化权重
    double weight_kinematics_turning_radius; //!< 强制实施最小转弯半径（类车机器人）的优化权重
    double weight_optimaltime; //!< 根据过渡（运行）时间压缩（缩短）轨迹的优化权重
    double weight_shortest_path; //!< 根据路径长度压缩（缩短）轨迹的优化权重
    double weight_obstacle; //!< 满足与障碍物最小间距的优化权重
    double weight_inflation; //!< 膨胀（缓冲区）惩罚的优化权重（应该设置得较小）
    double weight_dynamic_obstacle; //!< 满足与动态障碍物最小间距的优化权重
    double weight_dynamic_obstacle_inflation; //!< 动态障碍物膨胀（缓冲区）惩罚的优化权重（应该设置得较小）
    double weight_velocity_obstacle_ratio; //!< 根据与静态障碍物的距离满足最大允许速度的优化权重
    double weight_viapoint; //!< 最小化与途经点距离的优化权重
    double weight_prefer_rotdir; //!< 偏好特定转向方向的优化权重（-> 目前仅在检测到震荡时激活，参见 'oscillation_recovery'（震荡恢复））

    double weight_adapt_factor; //!< 某些特殊权重（目前是 'weight_obstacle'）在每次 TEB 外循环迭代中都会按此因子重复缩放（新权重 = 旧权重 * 因子）；迭代地增加权重，而不是先验地设置一个巨大的值，这能为底层优化问题带来更好的数值条件。
    double obstacle_cost_exponent; //!< 非线性障碍物代价的指数（代价 = 线性代价 * obstacle_cost_exponent）。设置为 1 可禁用非线性代价（默认值）
  } optim; //!< 优化相关参数


 struct HomotopyClasses
  {
    bool enable_homotopy_class_planning; //!< 激活同伦类规划（比简单规划需要多得多的资源，因为会同时优化多条轨迹）。
    bool enable_multithreading; //!< 激活多线程以并行规划多条轨迹。
    bool simple_exploration; //!< 如果为 true，则使用简单的左右方法（从障碍物的左侧或右侧通过）来探索不同的轨迹以生成路径，否则在起点和终点之间的指定区域内随机采样可能的路线图。
    int max_number_classes; //!< 指定允许的备选同伦类的最大数量（限制计算量）。
    int max_number_plans_in_current_class; //!< 指定尝试与当前轨迹处于同一同伦类的轨迹的最大数量（有助于避免陷入局部极小值）。
    double selection_cost_hysteresis; //!< 指定新候选轨迹相对于先前选定的轨迹在代价上必须降低多少才能被选中（如果 新代价 < 旧代价*因子，则选中）。
    double selection_prefer_initial_plan; //!< 为初始规划等价类中的轨迹指定一个 (0,1) 区间内的代价降低比例（即赋予初始路径所属同伦类的轨迹一定优势）。
    double selection_obst_cost_scale; //!< 仅用于选择“最佳”候选轨迹时对障碍物代价项进行的额外缩放。
    double selection_viapoint_cost_scale; //!< 仅用于选择“最佳”候选轨迹时对途经点代价项进行的额外缩放。
    bool selection_alternative_time_cost; //!< 如果为 true，则用总过渡时间替换时间代价。
    double selection_dropping_probability; //!< 在每个规划周期，除了当前“最佳”TEB之外的其它TEB将以该概率被随机丢弃。防止“死盯”在次优的备选同伦路径上。
    double switching_blocking_period; //!< 指定在允许切换到新的等价类之前需要经过的时间（以秒为单位）。

    int roadmap_graph_no_samples; //!< 如果关闭了 simple_exploration，则指定为创建路线图（roadmap graph）生成的采样点数量。
    double roadmap_graph_area_width; //!< 随机关键点/路径点是在起点和终点之间的矩形区域内采样的。指定该区域的宽度（以米为单位）。
    double roadmap_graph_area_length_scale; //!< 矩形区域的长度由起点和终点之间的距离决定。此参数进一步缩放该距离，以保持几何中心不变！
    double h_signature_prescaler; //!< 缩放障碍物的数值，以允许存在大量障碍物。不要选择极端低的值，否则障碍物之间将无法区分 (0.2<H<=1)。
    double h_signature_threshold; //!< 如果实部和虚部的差值都低于指定的阈值，则假定两个 h-签名 (h-signatures) 相等。

    double obstacle_keypoint_offset; //!< 如果开启了 simple_exploration，此参数决定了在障碍物左右两侧生成新关键点的距离（在 min_obstacle_dist 基础上额外增加的距离）。
    double obstacle_heading_threshold; //!< 指定障碍物朝向与目标朝向之间归一化标量积的值，以便在探索时考虑它们（障碍物）[0,1]。

    bool viapoints_all_candidates; //!< 如果为 true，不同拓扑结构的所有轨迹都将附加到当前途经点集合中，否则仅附加与初始/全局规划共享相同途经点的轨迹。

    bool visualize_hc_graph; //!< 可视化为了探索新同伦类而创建的图（graph）。
    double visualize_with_time_as_z_axis_scale; //!< 如果该值大于 0，则使用时间作为 z 轴（按此值缩放）在 3D 中可视化轨迹和障碍物。对动态障碍物最有用。
    bool delete_detours_backwards; //!< 如果启用，规划器将丢弃相对于最佳规划向后绕行（反向绕路）的规划。
    double detours_orientation_tolerance; //!< 如果规划的起始朝向与最佳规划的差异超过此值，则该规划被视为绕路（detour）。
    double length_start_orientation_vector; //!< 用于计算规划起始朝向的向量长度。
    double max_ratio_detours_duration_best_duration; //!< 如果绕路的执行时间 / 最佳 TEB 的执行时间 > 此值，则丢弃该绕路。
  } hcp;
  
 //! 恢复/备用策略相关参数
  struct Recovery
  {
    bool shrink_horizon_backup; //!< 允许规划器在自动检测到问题时，临时缩短规划时域（horizon）（缩短 50%）。
    double shrink_horizon_min_duration; //!< 在检测到不可行的轨迹时，指定缩短规划时域的最短持续时间。
    bool oscillation_recovery; //!< 尝试检测并解决同一等价类中多个解之间的震荡（机器人频繁在左/右/前/后之间来回切换）。
    double oscillation_v_eps; //!< 平均归一化线速度的阈值：如果 oscillation_v_eps 和 oscillation_omega_eps 两者都没有被超过，则检测到可能的震荡。
    double oscillation_omega_eps; //!< 平均归一化角速度的阈值：如果 oscillation_v_eps 和 oscillation_omega_eps 两者都没有被超过，则检测到可能的震荡。
    double oscillation_recovery_min_duration; //!< 检测到震荡后，激活恢复模式的最短持续时间 [秒]。
    double oscillation_filter_duration; //!< 用于检测震荡的滤波器长度/持续时间 [秒]。
    bool divergence_detection_enable; //!< 设为 True 以启用发散检测。
    int divergence_detection_max_chi_squared; //!< 可接受的最大马氏距离（Mahalanobis distance），超过此值则认为优化过程已经发散（失败）。
  } recovery; //!< 与恢复和备用策略相关的参数
  
  /**
  * @brief Construct the TebConfig using default values.
  * @warning If the \b rosparam server or/and \b dynamic_reconfigure (rqt_reconfigure) node are used,
  *	     the default variables will be overwritten: \n
  *	     E.g. if \e base_local_planner is utilized as plugin for the navigation stack, the initialize() method will register a
  * 	     dynamic_reconfigure server. A subset (not all but most) of the parameters are considered for dynamic modifications.
  * 	     All parameters considered by the dynamic_reconfigure server (and their \b default values) are
  * 	     set in \e PROJECT_SRC/cfg/TebLocalPlannerReconfigure.cfg. \n
  * 	     In addition the rosparam server can be queried to get parameters e.g. defiend in a launch file.
  * 	     The plugin source (or a possible binary source) can call loadRosParamFromNodeHandle() to update the parameters.
  * 	     In \e summary, default parameters are loaded in the following order (the right one overrides the left ones): \n
  * 		<b>TebConfig Constructor defaults << dynamic_reconfigure defaults << rosparam server defaults</b>
  */
  TebConfig()
  {

    odom_topic = "odom";
    map_frame = "odom";

    // Trajectory

    trajectory.teb_autosize = true;
    trajectory.dt_ref = 0.3;
    trajectory.dt_hysteresis = 0.1;
    trajectory.min_segment_time = 0.0;
    trajectory.max_segment_time = 0.0;
    trajectory.max_trajectory_time = 0.0;
    trajectory.weight_time_limit = 10.0;
    trajectory.jerk_limit_x = 5.0;
    trajectory.jerk_limit_y = 5.0;
    trajectory.jerk_limit_theta = 5.0;
    trajectory.weight_jerk = 0.0;
    trajectory.min_samples = 3;
    trajectory.max_samples = 500;
    trajectory.global_plan_overwrite_orientation = true;
    trajectory.follow_global_plan_orientation = false;
    trajectory.shortest_rotation_initialization = false;
    trajectory.translation_only = false;
    trajectory.goal_approach_enabled = false;
    trajectory.goal_approach_distance = 0.5;
    trajectory.goal_approach_max_speed = 0.5;
    trajectory.global_plan_orientation_tolerance = 0.1;
    trajectory.allow_init_with_backwards_motion = false;
    trajectory.global_plan_viapoint_sep = -1;
    trajectory.via_points_ordered = false;
    trajectory.max_global_plan_lookahead_dist = 1;
    trajectory.global_plan_prune_distance = 1;
    trajectory.exact_arc_length = false;
    trajectory.force_reinit_new_goal_dist = 1;
    trajectory.force_reinit_new_goal_angular = 0.5 * M_PI;
    trajectory.feasibility_check_no_poses = 5;
    trajectory.feasibility_check_lookahead_distance = -1;
    trajectory.publish_feedback = false;
    trajectory.min_resolution_collision_check_angular = M_PI;
    trajectory.control_look_ahead_poses = 1;
    
    // Robot

    robot.max_vel_x = 0.4;
    robot.max_vel_x_backwards = 0.2;
    robot.max_vel_y = 0.0;
    robot.max_vel_theta = 0.3;
    robot.base_max_vel_x = robot.max_vel_x;
    robot.base_max_vel_x_backwards = robot.base_max_vel_x_backwards;
    robot.base_max_vel_y = robot.base_max_vel_y;
    robot.base_max_vel_theta = robot.base_max_vel_theta;
    robot.acc_lim_x = 0.5;
    robot.acc_lim_y = 0.5;
    robot.acc_lim_theta = 0.5;
    robot.min_turning_radius = 0;
    robot.wheelbase = 1.0;
    robot.cmd_angle_instead_rotvel = false;
    robot.is_footprint_dynamic = false;
    robot.use_proportional_saturation = false;

    // GoalTolerance

    goal_tolerance.xy_goal_tolerance = 0.2;
    goal_tolerance.free_goal_vel = false;

    // Obstacles

    obstacles.min_obstacle_dist = 0.5;
    obstacles.inflation_dist = 0.6;
    obstacles.dynamic_obstacle_inflation_dist = 0.6;
    obstacles.include_dynamic_obstacles = true;
    obstacles.include_costmap_obstacles = true;
    obstacles.costmap_obstacles_behind_robot_dist = 1.5;
    obstacles.obstacle_poses_affected = 25;
    obstacles.legacy_obstacle_association = false;
    obstacles.obstacle_association_force_inclusion_factor = 1.5;
    obstacles.obstacle_association_cutoff_factor = 5;
    obstacles.costmap_converter_plugin = "";
    obstacles.costmap_converter_spin_thread = true;
    obstacles.costmap_converter_rate = 5;
    obstacles.obstacle_proximity_ratio_max_vel = 1;
    obstacles.obstacle_proximity_lower_bound = 0;
    obstacles.obstacle_proximity_upper_bound = 0.5;

    // Optimization

    optim.no_inner_iterations = 5;
    optim.no_outer_iterations = 4;
    optim.optimization_activate = true;
    optim.optimization_verbose = false;
    optim.penalty_epsilon = 0.05;
    optim.weight_max_vel_x = 2; //1
    optim.weight_max_vel_y = 2;
    optim.weight_max_vel_theta = 1;
    optim.weight_acc_lim_x = 1;
    optim.weight_acc_lim_y = 1;
    optim.weight_acc_lim_theta = 1;
    optim.weight_kinematics_nh = 1000;
    optim.weight_kinematics_forward_drive = 1;
    optim.weight_kinematics_turning_radius = 1;
    optim.weight_optimaltime = 1;
    optim.weight_shortest_path = 0;
    optim.weight_obstacle = 50;
    optim.weight_inflation = 0.1;
    optim.weight_dynamic_obstacle = 50;
    optim.weight_dynamic_obstacle_inflation = 0.1;
    optim.weight_velocity_obstacle_ratio = 0;
    optim.weight_viapoint = 1;
    optim.weight_prefer_rotdir = 50;

    optim.weight_adapt_factor = 2.0;
    optim.obstacle_cost_exponent = 1.0;

    // Homotopy Class Planner

    hcp.enable_homotopy_class_planning = true;
    hcp.enable_multithreading = true;
    hcp.simple_exploration = false;
    hcp.max_number_classes = 5;
    hcp.selection_cost_hysteresis = 1.0;
    hcp.selection_prefer_initial_plan = 0.95;
    hcp.selection_obst_cost_scale = 100.0;
    hcp.selection_viapoint_cost_scale = 1.0;
    hcp.selection_alternative_time_cost = false;
    hcp.selection_dropping_probability = 0.0;

    hcp.obstacle_keypoint_offset = 0.1;
    hcp.obstacle_heading_threshold = 0.45;
    hcp.roadmap_graph_no_samples = 15;
    hcp.roadmap_graph_area_width = 6; // [m]
    hcp.roadmap_graph_area_length_scale = 1.0;
    hcp.h_signature_prescaler = 1;
    hcp.h_signature_threshold = 0.1;
    hcp.switching_blocking_period = 0.0;

    hcp.viapoints_all_candidates = true;

    hcp.visualize_hc_graph = false;
    hcp.visualize_with_time_as_z_axis_scale = 0.0;
    hcp.delete_detours_backwards = true;
    hcp.detours_orientation_tolerance = M_PI / 2.0;
    hcp.length_start_orientation_vector = 0.4;
    hcp.max_ratio_detours_duration_best_duration = 3.0;

    // Recovery

    recovery.shrink_horizon_backup = true;
    recovery.shrink_horizon_min_duration = 10;
    recovery.oscillation_recovery = true;
    recovery.oscillation_v_eps = 0.1;
    recovery.oscillation_omega_eps = 0.1;
    recovery.oscillation_recovery_min_duration = 10;
    recovery.oscillation_filter_duration = 10;
    recovery.divergence_detection_enable = false;
    recovery.divergence_detection_max_chi_squared = 10;
  }
  
  void declareParameters(const nav2_util::LifecycleNode::SharedPtr, const std::string name);

  /**
   * @brief Load parmeters from the ros param server.
   * @param nh const reference to the local rclcpp::Node::SharedPtr
   */
  void loadRosParamFromNodeHandle(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name);
  
  /**
   * @brief Callback executed when a paramter change is detected
   * @param parameters list of changed parameters
   */
  rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
  
  /**
   * @brief Check parameters and print warnings in case of discrepancies
   *
   * Call this method whenever parameters are changed using public interfaces to inform the user
   * about some improper uses.
   */
  void checkParameters() const;
  
  /**
   * @brief Check if some deprecated parameters are found and print warnings
   * @param nh const reference to the local rclcpp::Node::SharedPtr
   */
  void checkDeprecated(const nav2_util::LifecycleNode::SharedPtr nh, const std::string name) const;
  
  /**
   * @brief Return the internal config mutex
   */
  std::mutex& configMutex() {return config_mutex_;}

private:
  std::mutex config_mutex_; //!< Mutex for config accesses and changes
  rclcpp::Logger logger_{rclcpp::get_logger("TEBLocalPlanner")};
};
} // namespace teb_local_planner

#endif
