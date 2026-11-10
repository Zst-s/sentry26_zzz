# 过洞决策与 USB 串口桥接

本工作区在原 Z-Localization 导航基础上接入 rm_bt/guodong.xml 和 decision/trans_node2。
导航、串口、决策由同一个 launch 管理；每次只启动一个串口进程。

## NUC 编译和运行

新增行为树依赖的 libzmq3-dev、libsqlite3-dev、ros-humble-ament-cmake-auto，以及原定位后端要求的 libnanoflann-dev、ros-humble-gtsam 4.2，需要在 NUC 上安装。BehaviorTree.CPP 4.6 和 ROS2 插件源码已包含在 src/bt 内。

~~~bash
sudo apt install libzmq3-dev libsqlite3-dev ros-humble-ament-cmake-auto libnanoflann-dev ros-humble-gtsam
~~~

在解压后的 sen_xin_z_cloud 工作区根目录运行：

~~~bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
./bt.sh
~~~

- ./bt.sh：启动原导航、USB串口和过洞决策。Nav2 action 可用且 bt_navigator 为 active 后，才启动行为树和计时器；等待超时120秒则不启动树。
- ./nav.sh：启动导航和USB串口，不启动行为树，便于先在RViz测试导航。
- ./guodong.sh：导航已经运行时，仅启动决策。不要在 ./bt.sh 运行期间再次执行。
- 默认设备 /dev/ttyACM0；切换设备：SERIAL_PORT=/dev/ttyACM1 ./bt.sh。
- WORLD 默认 footprint；可设置 WORLD、PRIOR_MAP_DIR、GRID_MAP_YAML 覆盖地图。
- 默认关闭 enable_fake_velocity 和 TEB translation_only，TEB 与 Nav2 终点朝向容差均为0.15rad，以执行目标四元数朝向；ENABLE_FAKE_VELOCITY=true ./bt.sh 可恢复原虚拟朝向模式，但该模式会忽略终点朝向。
- 仅启动导航而不访问串口：ENABLE_SERIAL=false ./nav.sh。
- 不要同时启动旧 trans_node/trans_node4 或独立收头测试程序。
- 配置：src/decision/config/trans_node2.yaml；行为树：src/bt/rm_bt/behavior_tree/guodong.xml。

## 数据链路

guodong.xml/Nav2Pose -> /navigate_to_pose -> 原Nav2 -> /cmd_vel -> trans_node2 -> USB
guodong.xml/PreparePassage -> /if_guodong -> trans_node2 -> 同一个USB帧

trans_node2 默认 MOVE姿态3、nav_ready=1、if_guodong=1（抬头）。
串口100Hz持续发送；速度指令超过0.25秒未更新或里程计超过0.25秒未更新时发送零速度。
导航包的 zlio_nav_adapter 提供 /base_imu（odom坐标系），launch 将其连接到串口节点的里程计检查。
这里不使用旧工程特有的 /pointlio/status 健康话题；原导航的定位处理仍由原导航负责。
不启动旧 rosout_node，避免其通过终端日志推断 nav_ready 并误关断速度。

## 已确认的线上格式

完整148字节，小端。第0字节A5，第1字节94（十六进制，即总长148），第2字节起146字节packed结构体。

| 字段 | 帧内偏移，从0开始 |
| --- | --- |
| Navigation_x = vx | 2，float32 |
| Navigation_y = vy | 6，float32 |
| Navigation_z = wz | 10，float32 |
| target_stance | 14 |
| nav_ready | 15 |
| super_cap_flag | 16 |
| if_guodong：0收头、1抬头 | 17 |
| if_stronger | 18 |

使用 rm_serial_protocol/nav_rx.hpp 编码。暂未使用的结构体尾部清零，未生成裁判CRC。
保留原30字节下位机反馈解析和持续读取线程。USB接收需按帧长累积，不能把每次USB分包当作完整帧。

## 原行为树的实际执行逻辑

此次迁移保留 guodong.xml 中的计时条件、分支顺序、目标点和Delay。
第二个目标仍为(2.2, 2.766)，四元数(0,0,-0.70710678,0.70710678)，对应地图yaw=-90度。
这是绝对地图朝向，不是运行时在当前机器人朝向上再旋转90度。

注意：这棵树仍按计时运行，不是按洞口区域边界触发。第一次tick时6秒/15秒条件均不满足，会进入第三分支发送0（收头），随后Delay等待50秒。
6秒条件所在分支优先于15秒分支；当6秒分支返回RUNNING或SUCCESS时，15秒分支不会执行。
这些是原树现有逻辑，桥接没有把它改成新的区域决策。

## 诊断

~~~bash
ros2 action list
ros2 lifecycle get /bt_navigator
ros2 topic echo /cmd_vel
ros2 topic echo /if_guodong --qos-durability transient_local
ros2 topic echo /passage/serial_tx_status
~~~

serial_tx_status 中 frame_vx/frame_vy/frame_wz 是实际打包的速度，blocking_reason 表示速度放行原因；
written_bytes=148 表示写入驱动的完整帧字节数，rx_bytes_total 表示累计回读数据量。
这些不能替代下位机接收确认。

编译和软件验证记录见 BUILD_BRIDGE_REPORT.md。
