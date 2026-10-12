# sen_xin：2027 地形感知接入

本次只接入 `/home/zst/2027-sentry-navi/src/perception/rog_map` 的概率体素、射线更新、地形分类、滚动窗口和障碍衰减。原项目把 ROGMap 放在 MINCO 规划器内部，因此这里新增独立节点和 Nav2 插件，**没有复制 MINCO/MPC**。

## 包和功能的边界

|功能|本次处理|
|---|---|
|`rog_map`|原样复制来源库及其版权注释；不改算法实现|
|`sentry_terrain_bridge`|新增独立运行、消息校验、OccupancyGrid 转换及 Nav2 适配|
|FAST-LIO / Point-LIO|保留本车的源码、IMU 外参、估计器和 TF；只利用已有点云输出|
|Livox 驱动|保留本车驱动/IP，不复制来源双雷达配置|
|SLAM Toolbox / AMCL / ICP / map_server|保留；地图路径和选择规则不变|
|Navfn / TEB / 速度平滑 / 串口 / decision|保留源码和参数；不添加速度发布者|
|linefit / pointcloud_to_laserscan|导航定位以及 `shadow`、`legacy` 建图继续使用；`rog_map` 建图时停用|
|`msg_convert` / DBSCAN / 双雷达标定|不复制未启用的实验流程和双雷达工具；ROGMap 自带裁剪功能暂关闭|

注意：这不是把旧感知相关目录全部删除。导航定位仍保留原 `/scan`，因为现有 AMCL 和 SLAM Toolbox 定位配置尚未切到 ROGMap。建图模式的输入已单独改为 `/rog_map/scan`；全局位姿图和 `/map` 仍由 SLAM Toolbox 负责，不要把 ROGMap 的局部 `/terrain/grid` 误认为可保存的全局图。

## 数据链路与坐标

```text
原 Livox → 原 FAST-LIO → /cloud_registered_body（密集、IMU-body 数值）
                     → /Odometry（odom 中的 IMU 状态）
                                  ↓ 配准与时间校验
                              独立 ROGMap
                                  ↓
                         /terrain/grid（odom）
                                  ↓ 当前 map←odom TF
                        Nav2 TerrainLayer → 原 Navfn / TEB
```

`mode:=mapping perception:=rog_map` 时另有一条建图链路：

```text
ROGMap /terrain/grid（局部）→ 从雷达当前位置逐束射线投影
                            → /rog_map/scan（LaserScan）
                            → SLAM Toolbox → /map、位姿图、map→odom
```

在这个组合下，旧 linefit 与 pointcloud_to_laserscan 不启动；`shadow`、`legacy` 建图仍使用旧 `/scan`。新扫描只把明确分类为障碍的体素列变成量测终点；未知格子给 NaN，不把未知伪装成自由空间。扫描来自局部滚动栅格而非单帧原始回波，因此它不是独立的全局 SLAM，也不保证墙上每个入口一定能被识别。必须在 NUC 静止和低速条件下核验时间、TF、入口两侧墙体与中间间隙，再决定是否用它保存正式地图。

经核对本仓库 `publish_frame_body()`：FAST-LIO 的 body 点云已经应用雷达到 IMU 的外参，**数值在 IMU 系，但原消息 header 写的是 `livox_frame`**。适配器显式区分 `cloud_coordinates: imu`，用 `/Odometry` 的 IMU 位姿变换一次，不再乘雷达外参，也不通过这个标签再查一次 TF。雷达射线起点单独使用 `p_imu + R_imu * extrinsic_T`。外参由 launch 从所选 LIO 配置读取。

旧 Point-LIO 的 body 输出未做逐点运动补偿，所以该分支改用 `/cloud_registered`（已经逐点配准到 odom），`cloud_coordinates: odom`，不会再变换点坐标。但旧算法输出经过约 0.5 m 降采样，**细小障碍可能丢失**；该分支仅完成接口适配，必须先旁路验证。不要为获得密集点云直接把未经补偿的 body 输出整体变换。若需要来源项目 Point-LIO 的密集逐点输出，应另行移植那部分输出机制并测试。

本次不发布 `map→odom`、`odom→base_link` 或任何静态 TF。ROGMap 的先验地图融合关闭，地图继续归原定位模块管理。没有 `map→odom` 时新代价层会报警并保持非实时，不会用假静态 TF 掩盖错误。

## 三种运行方式

以下示例需要在运行设备上先编译并 source 新安装目录。本次只完成了当前机器上的隔离构建和单元测试，没有运行实车 launch 或启动串口。使用你自己的地图名称，示例中的 `my_map11` 不代表地图已经核验。

```bash
cd /home/zst/zzs  # 当前电脑上的目录；NUC 上按实际工作区路径替换。
source install/setup.bash

# 建议第一次先旁路比较：原避障工作，新算法仅发布地形供观察。
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=nav world:=my_map11 lio:=fastlio localization:=icp perception:=shadow

# 核验合格后，新地形接入局部/全局代价地图。
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=nav world:=my_map11 lio:=fastlio localization:=icp perception:=rog_map

# 切回原实时避障和原 Nav2 配置，不需要删除新功能包。
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=nav world:=my_map11 lio:=fastlio localization:=icp perception:=legacy
```

`bringup_real.launch.py` 默认 `perception:=rog_map`；现有 `nav.sh` 未改动，重编译后仍调用该 launch 时会采用新默认值。**第一次不要直接用会启动串口的 nav.sh 上车运动**；先单独启动 `perception:=shadow`，保持底盘驱动禁止输出，并准备急停。

仿真 launch、本车地图保存脚本和 sen_xin11 均未改。此接入暂仅覆盖实车启动文件，不能声称 Gazebo 已验证。

## 如何检查

建图试跑（底盘先禁止自主运动）：

```bash
ros2 launch rm_nav_bringup bringup_real.launch.py mode:=mapping lio:=fastlio perception:=rog_map
ros2 topic hz /rog_map/scan
ros2 param get /slam_toolbox scan_topic
```

最后一条应返回 `/rog_map/scan`。RViz 分别显示 `/terrain/grid`、`/rog_map/scan`、`/map`，重点观察入口两侧墙点及中间空隙。若新扫描漏墙或 TF 不通，改用 `perception:=legacy` 回到原建图输入，不要在未确认时直接保存并用于实车导航。

RViz 的 Fixed Frame 使用现有的 `map`（TF 正常时）或用 `odom` 单独看感知。添加 Map 显示，Topic 选 `/terrain/grid`，Durability 用 Volatile。不要把这个话题替换成定位用的 `/map`。

```bash
ros2 topic hz /terrain/grid
ros2 topic echo /terrain/grid --once --field header
ros2 topic info /cloud_registered_body -v
ros2 topic info /Odometry -v
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /global_costmap/global_costmap plugins
ros2 run tf2_ros tf2_echo map odom
```

实际节点名有命名空间时以 `ros2 node list` 为准。rog_map 模式局部插件列表应为 terrain_layer、inflation_layer；全局应为 static_layer、terrain_layer、inflation_layer。先比较静止的墙、桌脚、低矮箱体，再在断开电机输出的条件下人工移动/缓慢转动车，检查障碍是否跟随错位、出现延迟或消失。不做高速自主导航验收。

输出 OccupancyGrid 正确区分：未知=-1、自由=0、障碍=100；不能把原库的 254/255 直接塞入 int8 栅格。新插件按最大代价叠加，不用观测自由空间擦掉先验地图的墙。输入超过 `max_input_age` 不处理；投影/TF 超过 `data_timeout`，代价层标记 `current=false`。这不是硬件急停，不能代替底盘命令超时保护。

## 参数与已知限制

- 配置在 `src/rm_nav_bringup/config/reality/rog_map_real.yaml`，新 Nav2 参数在 `nav2_params_rog_map_real.yaml`。原 `nav2_params_real.yaml` 原样保留。之后如果调 TEB 等公共参数，需同步两个 Nav2 文件；可运行下面的静态检查识别意外分歧。
- 来源项目的地形阈值（例如 surface_height_delta_max=0.25）是**一列体素的高度差分类门限，不是允许本车跨过 25 cm 台阶的保证**。你的车身高度、离地间隙、地面 Z、细墙和斜坡识别必须重新测量。
- `scan_z_min_abs/max_abs` 是 odom 中绝对高度。初始地面高度或爬坡改变后，固定高度范围可能裁掉地形；不要直接把 sensor_height 当成这两个参数。
- 悬崖/负障碍、遮挡、薄平面物体可能漏检。该接入没有新增悬崖检测或功能安全保护。
- 来源地图特定剔除框、双雷达偏移、先验图融合全部关闭。距离场未供现有 TEB/Navfn 使用，默认关闭以节省 CPU；现有 inflation_layer 保持不变。
- 只缓存最新点云，里程计按时间戳找近邻（默认允许 0.1 s）；这不是重新做 LIO 的运动补偿，也不能修复 LIO 漂移。时间重置后应一起重启感知和相关定位节点。
- ROGMap 累积多帧再衰减，瞬时遮挡后的保留/消失需要实测；不是单帧点云分类器。
- 约 10×10×3 m、5 cm 分辨率会增加 NUC 负载；目前未测 CPU、内存、端到端时延。
- `/cloud_registered`、原 LIO 估计器与串口节点未修改。FAST-LIO 仅确保其已有 body 输出打开，不更改原 world 输出密度。

## 验证与构建边界

```bash
python3 src/rm_perception/sentry_terrain_bridge/test/check_configuration.py /home/zst/zzs
```

这是只读静态检查。本次在当前机器的 `/tmp` 隔离目录完成 `rog_map`、`sentry_terrain_bridge`、`rm_nav_bringup` 构建，新增的扫描投影单元测试通过；没有覆盖工作区现有 build/install/log，也没有在 NUC 上运行。仍需在 NUC 编译并测试消息、TF、入口分类、停止响应后，才能判定新建图输入可用于正式建图或实车导航。
