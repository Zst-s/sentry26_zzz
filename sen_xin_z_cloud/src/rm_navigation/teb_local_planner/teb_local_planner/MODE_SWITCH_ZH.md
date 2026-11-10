# TEB 运行中切换速度和优化权重

当前实车配置已使用 `/home/lyg/sen_xin/src/rm_nav_bringup/config/reality/through_hole.yaml`：第一套 `default` 为启动时现有参数；第二套 `slow` 为占位参数，前进/横移上限均为 0.3 m/s，碰撞轮廓沿用当前四个 ±0.28 m 顶点。修改第二套就在这个文件里改。部署到另一台电脑时，需要修改导航 YAML 中 `mode_profiles_file` 的绝对路径。

`footprint` 现在也可作为模式字段，格式为 `[[x,y], ...]`，顶点沿边界顺序填写，必须是包含车身原点的凸多边形。它同步更新局部 costmap 轮廓和 TEB 最终碰撞检查轮廓；保留局部 costmap 现有 padding。第一套保存未加 padding 的启动轮廓，恢复时不会重复加 padding。这个切换无需打开 `is_footprint_dynamic`；该开关原有的反向赋值错误也已修复。

按本次需求，只切换碰撞检查轮廓；`FollowPath.footprint_model` 的优化模型和全局 costmap 轮廓保持原设置。因此调整检查轮廓后，优化避障间距仍取决于原优化模型，不代表车一定会走进原来避开的通道。局部 costmap 的膨胀结果会在后续地图更新周期刷新。

TEB 自己订阅 `std_msgs/msg/String` 模式话题。启动配置时读取一个包含多套参数的 YAML；回调只记录最新模式名，下一轮 `computeVelocityCommands()` 开始、持有配置锁后统一应用。切换不重读文件、不调用参数服务、不清空轨迹，也不取消或重新发送目标。

这不是硬实时保证：如果本轮优化已经开始，请求要等下一轮；延迟还包括消息调度和实际控制周期。没有导航任务时请求暂存，到下一次规划时应用。

## 启用

编译更新后的 `teb_local_planner`，加载同一工作空间环境，然后在实际加载的导航 YAML 中设置：

```yaml
controller_server:
  ros__parameters:
    FollowPath:
      mode_profiles_file: /你的绝对路径/mode_profiles.yaml
      mode_topic: /teb_mode
```

`mode_profiles_file` 默认空字符串，此时不订阅模式话题，保持原参数运行。模式文件路径和话题名只在配置阶段读取。若通过 fake 启动入口生成导航 YAML，上述两个字段也会保留。

已提供 `params/mode_profiles.yaml`，安装后在 `$(ros2 pkg prefix teb_local_planner)/share/teb_local_planner/params/mode_profiles.yaml`。这些是示例值，按车辆修改后再启用。普通 YAML 字符串中的 `$(...)` 不会自动执行，必须填写展开后的绝对路径。

模式文件格式：

```yaml
profiles:
  slow:
    max_vel_x: 0.3
    max_vel_y: 0.3
    weight_obstacle: 50.0
  cautious:
    max_vel_x: 0.2
    max_vel_y: 0.2
    weight_obstacle: 100.0
```

每套模式独立继承启动时的基线，未写的参数使用基线，不继承上一个模式。例如从 slow 切到 cautious，slow 特有的覆盖不会残留。内置 `default` 恢复启动时的全部可切换参数，不能在文件中重定义。

## 发送与确认

```bash
ros2 topic pub --once /teb_mode std_msgs/msg/String "{data: slow}"
ros2 topic pub --once /teb_mode std_msgs/msg/String "{data: cautious}"
ros2 topic pub --once /teb_mode std_msgs/msg/String "{data: default}"
```

确认已生效的模式：

```bash
ros2 topic echo /teb_mode/active --qos-durability transient_local
```

实际应用后打印 `TEB mode applied: slow`，并发布 active。它表示本轮已采用这套配置，不表示轨迹一定可行或车已经执行速度。配置错误会使配置阶段失败并给出原因；运行中未知模式只警告并保持当前模式。突发消息只保留最后收到的有效请求，相同模式不会反复应用。

这是优化器内部配置切换，**不会写回 ROS 参数存储**，所以 `ros2 param get` 显示的仍可能是启动值。用 active 话题确认模式。启用后不要同时通过参数服务修改这些字段，否则可覆盖当前配置；下次切换模式又会恢复预加载的模式值。

## 支持的字段

- 速度：`max_vel_x`、`max_vel_x_backwards`、`max_vel_y`、`max_vel_theta`。
- 加速度：`acc_lim_x`、`acc_lim_y`、`acc_lim_theta`。
- 权重：`weight_max_vel_x/y/theta`、`weight_acc_lim_x/y/theta`（这些缩写对应三个独立字段），`weight_kinematics_nh`、`weight_kinematics_forward_drive`、`weight_kinematics_turning_radius`、`weight_optimaltime`、`weight_shortest_path`、`weight_obstacle`、`weight_inflation`、`weight_dynamic_obstacle`、`weight_dynamic_obstacle_inflation`、`weight_velocity_obstacle_ratio`、`weight_viapoint`、`weight_prefer_rotdir`。

所有标量数值必须有限且非负；前进/角速度上限及加速度上限必须大于零。横移速度不能从零变正或从正变零，这会改变运动学能力。`footprint` 单独按多边形规则检查。其他字段一律拒绝，不能用这里切换 TF、插件或碰撞检查范围。

外部 Nav2 限速会在模式速度基础上继续生效，不会被模式切换绕过。原来 `setSpeedLimit()` 错把缩放值写回 base 上限，现已改为保留基准并更新有效上限，清除限速能恢复当前模式速度。

TEB 的加速度和权重仍属于优化约束，并不等同于输出端的硬加速度限制。切换权重可能使轨迹改变；如果需要严格限制指令变化，应继续保留速度平滑器。碰撞检查逻辑不因模式切换而关闭。

## 验证

`test/mode_profiles_test.cpp` 检查基线恢复、非法字段/数值、运动学切换拒绝、重复模式以及文件失败时不部分提交。

2026-11-03：TEB 已在 `mode_check_build` 中编译通过。配置校验测试、ROS 话题切换测试以及原有 translation_only、goal_approach、shortest_rotation 测试均通过。话题测试验证了收到消息后仍保持旧配置、周期边界统一应用、删除模式文件后仍可切换，以及跨模式保留/解除外部限速。尚未进行实车切换测试；此独立测试构建不会自动替换实车正在使用的安装目录。
