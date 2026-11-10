# 云台巡航

启动后持续转圈。`/enemy_pose` 的 `orientation` 是 map 坐标系下希望云台转到的朝向（不使用 position）。收到后按最短方向转到该朝向，再以该朝向为中心左右各 45° 往复。新敌人消息会重新指定中心。`/find_already` 收到 true 后立即发布零速度并保持停止，期间忽略敌人消息；`/lose_already` 收到 true 后重新持续转圈，等待下一条敌人消息。两个 Bool 话题的 false 不触发动作，true 按事件处理，不应持续重复发送丢失事件。

## 编译与启动

```bash
cd /home/lyg/sen_xin
source /opt/ros/humble/setup.bash
colcon build --packages-select cruise --symlink-install
source install/setup.bash
ros2 launch cruise cruise.launch.py
```

也可以指定源目录配置，避免修改后使用到旧的安装副本：

```bash
ros2 launch cruise cruise.launch.py params_file:=/home/lyg/sen_xin/src/cruise/config/cruise.yaml
```

这只启动巡航节点，不会再启动一套导航。定位系统需要单独运行。速度在 `config/cruise.yaml` 的 `cruise_speed` 修改，单位 rad/s；配置在启动时加载，改文件后重启节点。靠近目标或扫动端点时，按 `yaw_gain` 比例减速，`yaw_tolerance` 内认为到位。

## 坐标系及输出

工程 FAST_LIO 提供 `odom → base_link`，URDF 提供 `base_link → livox_frame`，定位提供 `map → odom`。本节点添加静态 `livox_frame → gimbal`，并读取完整的 `map → gimbal` 来得到当前云台朝向，不订阅额外的当前角度话题。

`lidar_to_gimbal_xyz` 表示 gimbal 原点在雷达系中的位置，单位米；`lidar_to_gimbal_rpy` 表示 gimbal 相对雷达的旋转，单位弧度。暂填零，需要按照安装关系设置。使用静态变换的前提是雷达与云台刚性固定。本实现是水平 yaw 巡航，假定云台 +Z 为竖直转轴；不适用于雷达与云台存在相对转动、或任意倾斜转轴的情况。其他节点不要重复发布同名 gimbal 子坐标系。

输出 `/cruise_speed` 类型为 **geometry_msgs/msg/Twist**，仅 `angular.z` 有值，绕 gimbal +Z 逆时针为正，单位 rad/s。Twist 不带 frame_id，这里的 gimbal 是约定的参考坐标系。草稿中的 RobotStatus 是血量等状态消息，不包含角速度，所以已改为 Twist。

本节点不控制串口、不发送 `/cmd_vel`，下位机接收模块需要订阅 `/cruise_speed` 并取 angular.z。收到零必须停止巡航；节点意外退出无法保证最后一条零消息送达，接收端应设置速度消息超时停止。find 停止的是本节点巡航输出，不会停止另一套自瞄控制。

## 输入示例

将云台转到 map 下 90° 朝向，再开始左右扫描：

```bash
ros2 topic pub --once /enemy_pose geometry_msgs/msg/Pose "{orientation: {z: 0.70710678, w: 0.70710678}}"
ros2 run cruise send_cruise_event.py find
ros2 run cruise send_cruise_event.py lose
ros2 topic echo /cruise_speed
```

事件发送工具先确认订阅者是巡航节点，再重复发送直到收到 `/cruise/found_state` 确认；普通 `ros2 topic pub --once` 只发布一条且默认很快退出，即使没有录包器，也不能以命令行的 `publishing #1` 判断巡航回调已执行。巡航节点收到事件时会写入日志，可用来区分消息漏收和后续速度链路问题。`lose` 被确认只表示状态已恢复；TF 不可用时速度仍会是零。

Pose 没有 header，因此发布者必须先把朝向转换到 map 系。必须提供有效四元数，不能用四个零。默认话题名均可在 YAML 修改。

TF 未连接、无效或超时会持续输出零，并每两秒最多打印一次原因。TF 恢复后继续原状态，不自动取消 find 的停止锁定。
