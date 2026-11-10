# 全向车最短转向初始化

用于修复全局路径中间 yaw=0 将 TEB 初始化到绕远转向的问题。例如日志 cycle=200：
起点 yaw=2.29348621、终点 yaw=-3.01187491。经过中间 yaw=0 的初始轨迹累计转向约
304°；新的初始轨迹按归一化后的起终点转角 +56°，沿路径长度分配中间朝向。

## 配置

参数放在 controller_server 的 FollowPath 下：

```yaml
shortest_rotation_initialization: true
follow_global_plan_orientation: false
global_plan_overwrite_orientation: false
enable_homotopy_class_planning: false
```

代码默认关闭，当前 reality/nav2_params_real.yaml 已开启。要求全向模型 max_vel_y>0。
此模式不能与严格跟随全局中间朝向同时开启；当前仅支持单轨迹规划。
将 shortest_rotation_initialization 改为 false 即可恢复原初始化方式。

同步源码到另一台车后需要重新编译 teb_local_planner 和 rm_nav_bringup，并重启导航。
已经加载新版插件时也可以切换：

```bash
ros2 param set /controller_server FollowPath.shortest_rotation_initialization true
ros2 param get /controller_server FollowPath.shortest_rotation_initialization
```

## 行为与验证

- 只修改初始猜测中的中间朝向，路径位置、实测起点和目标朝向保持原值。
- 开关变化后，在下次规划时丢弃旧轨迹，避免沿用旧的长转向分支。
- 正常热启动继续复用优化轨迹；重新初始化时使用较短转向。
- 优化、角速度/加速度限制、碰撞检查继续执行。此功能不强制最终轨迹只能按最短角转向。
- 诊断 input 的 initialization 将出现 shortest_rotation_fresh 或
  shortest_rotation_goal_reinit；reference_pose 仍记录原始输入，pose 记录修改后的初始轨迹。
- 比较同一周期 input 和最终 after 的累计转角，不能只做终点 yaw 减起点 yaw：
  应逐段归一化角差到 [-pi, pi] 后再累加。

如果底盘不执行 angular.z，这个功能只减少多余转向，不能消除控制器要求转向但车辆不执行的问题。

测试覆盖日志中的 304°→56°、跨 ±pi 的两个方向、非均匀/重复位置、原地旋转、
短路径和 TimedElasticBand 的自动增删轨迹点。运行：

```bash
ctest --test-dir /home/lyg/sen_xin/build/teb_local_planner --output-on-failure
```
