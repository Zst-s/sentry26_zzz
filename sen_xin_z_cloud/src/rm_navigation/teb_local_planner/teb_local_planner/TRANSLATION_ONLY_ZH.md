# 只平移、固定底盘朝向模式

`FollowPath.translation_only` 默认 false。启用后，每轮规划用实测起点 yaw 作为该轮所有轨迹点的 yaw；不采用全局目标箭头朝向。每次优化更新仅允许修改 X/Y，禁止修改 yaw；自动增删轨迹点后也重新锁定，时间间隔仍参与优化。

```yaml
FollowPath:
  translation_only: true
  follow_global_plan_orientation: false
  enable_homotopy_class_planning: false
  # max_vel_y 必须大于零，并按真实底盘能力设置
```

本地实车 YAML 保持 false，避免未经验证直接启用。停车、取消导航任务后可动态切换：

```bash
ros2 param set /controller_server FollowPath.translation_only true
ros2 param set /controller_server FollowPath.translation_only false
```

模式切换时清除旧轨迹，避免复用带旋转的旧解。启用时 shortest_rotation_initialization 和 global_plan_overwrite_orientation 被覆盖；严格跟随全局朝向、同伦多路径、阿克曼转向输出不兼容，会拒绝规划。差速/汽车式运动学边和旋转方向偏好边在该模式下不加入，保留全向速度、平移加速度、障碍、引导点、时间代价等。边界角速度按零建模，输出 angular.z 为零。

障碍物距离计算和最终 costmap 轮廓碰撞检查继续使用实际配置的 footprint。不能以关闭碰撞检查来穿过必须旋转才能通过的通道。只平移并不保证路径最短、无横向振荡或没有低速死区。

## 使用前必须确认

1. base_link 必须代表真实底盘。若雷达装在旋转云台上，应有正确的底盘—云台—雷达动态 TF；本次改动不修复 TF 或速度坐标变换。
2. 该模式不是主动保持航向控制：每轮重新使用实测 yaw，不会发角速度把被扰动的底盘转回旧朝向。下位机若需要保持底盘航向，应独立实现。
3. Nav2 到达检查器仍独立执行，TEB 不会改它。对只要求位置的任务，使用当前版本支持的位置检查器，或在 SimpleGoalChecker 中将 yaw_goal_tolerance 设置为不小于 pi（例如 3.141593）；保留用户要求的 xy_goal_tolerance。确认运行值，不要只看文件。
4. Nav2 行为树的 Spin 恢复、其他速度发布节点不由本开关控制；机器人全系统禁止旋转时，还需核对任务/恢复行为。
5. 保持 max_vel_theta、acc_lim_theta 为正的正常数值，不要通过置零来启用模式。
6. 先在无障碍安全区域验证横移方向，再进行障碍测试。关闭 goal_approach_enabled 可单独评估本模式，不叠加末端限速。

## 验证

诊断 details 新增 translation_only=1。所有 pose 的 yaw 应在一轮中相同，输出 angular.z 应为零；X/Y 和 dt 仍会改变。默认开关关闭时保留原有优化。

单元/优化器测试覆盖：yaw 锁定时 X/Y 可更新、解锁恢复、存在障碍时真实 g2o 优化、自动重采样、热启动跟随新实测起点朝向、禁用后恢复目标朝向。尚未实车验证。
