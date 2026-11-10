# 桥接构建与验证记录

日期：2026-11-07（北京时间）。

## 构建结果

ROS 2 Humble / Ubuntu 22.04 Docker容器 senxin_bridge_build2_20261107。
完整工作区 28 个包全部编译成功，退出码0。完整最终日志：verification/bridge_build.log。
全部源码来自本目录 /bridge；复用现有 /ws/install_humble 的外部依赖环境与 Livox SDK。
每个本工作区包均已从本目录构建，最终一次为在前述完整构建基础上的增量检查。

构建命令：

~~~bash
source /opt/ros/humble/setup.bash
source /ws/install_humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
colcon build --executor sequential --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
~~~

容器补装 libnanoflann-dev 1.4.2+ds-1、ros-humble-gtsam 4.2.0。
原 decision 旧节点仍有非致命编译警告；新启动入口选择 trans_node2。

## 桥接内容

- 原导航输出 /cmd_vel；Nav2Pose使用 /navigate_to_pose。
- PreparePassage通过 /if_guodong 控制同一个 trans_node2：0收头、1抬头。
- A5、总长148、146字节packed结构体；if_guodong帧内偏移17。
- 原始字节编码器 rm_serial_protocol、C++串口实现和行为树插件源码已纳入工作区。
- 新 launch 管理原导航、串口和可选决策；不并发启动旧串口节点。
- 默认 /dev/ttyACM0；里程计检查接 zlio_nav_adapter 的 /base_imu。
- 关闭原导航虚拟朝向和TEB仅平移模式；TEB及Nav2朝向容差均为0.15rad。
- Nav2动作服务可用且bt_navigator为active后才启动树、开始计时。

## 原代码的 Humble 兼容修正

- rm_path_recovery/src/path_translate.cpp：在configure阶段从生命周期节点取得ROS时钟。
- backend/apps/publisher.hpp、backend/tools/calibrate.cpp：使用Humble提供的tf2_ros头文件后缀。
- sentry_terrain_bridge/src/paired_static_layer.cpp：读取同名footprint_clearing_enabled参数，
  保留不允许开启footprint清除的原检查，兼容Humble不存在该基类成员的情况。

## 软件验证

- 原协议C++布局检查通过：148字节、所有字段偏移、小端浮点/整型、数组及控制前缀长度。
- 正式串口PTY测试两种模式均通过：非零速度转发、0/1切换、默认抬头、
  持续回读反馈、250ms指令超时归零、nav_ready拦截、非有限数拒绝、里程计超时拦截。
- 启动等待测试通过：Nav2 inactive时不tick树、不发收头；active后才启动原树。
- 无Nav2时等待超时退出，不启动树。
- ros2 launch --show-args、Shell/Python/XML语法、安装XML一致性和关键二进制ldd检查通过。
- 上述测试使用隔离ROS域和虚拟串口，未连接实车，未验证实际运动或定位效果。

## 行为树范围

保留原计时树的分支、6秒/15秒门限、50秒/80秒Delay和三个目标点。
第二个目标四元数仍为(0,0,-0.70710678,0.70710678)，为地图绝对yaw=-90度。
原树第一次tick即发送收头0；6秒分支优先于15秒分支，具体说明见README_PASSAGE_BRIDGE.md。
本次完成桥接，没有新增洞口区域边界触发决策。
