# 实机启动（Z-Localization）

`bringup_real.launch.py` 使用 ZLIO 前端和 backend 后端，已移除旧 FAST-LIO、Point-LIO、ICP、AMCL 启动分支。

TF 链为 `map → odom → imu → base_link → livox_frame`。后端只发布 `map → odom`，前端发布 `odom → imu`。
`imu → base_link` 由当前 `urdf/sentry_robot_real.xacro` 和前端配置的 `sensor.body_from_lidar` 联合计算。
后端车体外参同步采用这个结果，关闭额外的 `map → body` TF。
当前 URDF 的雷达安装位置是 `[0, 0.17, 0.25] m`；旧 `measurement_params_real.yaml` 中的数值并未被该 xacro 使用。
请按本车实际标定修改 URDF 和前端配置。

`zlio_nav_adapter.py` 将 `/Odometry` 的 IMU 位姿、速度及协方差转换到车体原点，发布 `/base_imu` 和水平 `base_footprint` TF。
转换包含旋转时的杆臂速度。驱动使用 `xfer_format=1`，ZLIO 直接读取带逐点时间戳的 CustomMsg；
地面分割和 ROGMap 使用 ZLIO 发布的 `/frontend/cloud_deskewed`。

## 编译

仓库有两份同名 `livox_ros_driver2`，不能同时参与 colcon 构建。以下命令只选择 Z-Localization 内的驱动：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --base-paths \
  src/z-localization/src src/rm_nav_bringup src/rm_navigation src/rm_perception \
  src/decision src/rm_decision_interfaces src/base_interfaces_demo
source install/setup.bash
```

需要先安装 Z-Localization 的 GTSAM、Livox-SDK2 等依赖，见其 README。

## 建图

```bash
ros2 launch rm_nav_bringup bringup_real.launch.py \
  mode:=mapping world:=site lio_rviz:=true \
  save_directory:="$PWD/maps/site_raw"
```

建图模式启动前后端、地面分割、激光扫描和 slam_toolbox，不启动 Nav2 控制器。
slam_toolbox 使用后端的 map 系位姿生成二维栅格，关闭自身 TF 发布、扫描匹配和回环，避免两套定位争抢 TF。
若后端回环优化、UWB 校正或离线地图精修改变了历史位姿，二维栅格不会随之重新优化；
最终使用前需要检查二维图与先验地图包坐标是否对齐，必要时用最终位姿重新生成二维图。

保存当前 mapping 地图（`WORLD` 与地图名一致）：

```bash
./save_mapping_map.sh site
```

脚本调用 `/stop` 和 ZLIO 保存服务，保存 SLAM posegraph，并把经过
`/passage/verified_static_clearing` 与 `/passage/memory_status` 校验的二维图写到
`src/rm_nav_bringup/map/site.yaml`，原始图另存为 `site_raw.yaml`；ZLIO 先验包复制到
`src/rm_nav_bringup/PCD/site/`。证据不匹配或过期时拒绝导出，不会自动清除二维图障碍。
此修正仅针对二维栅格的已证实通道格，不会修改 ZLIO 点云地图包。
若尚未形成有效通道证据，修正图与原始图相同；可检查
`/passage/memory_status` 的 `state`、`map_associated` 和
`static_output_cleared_cells`。
若建图期间后端改变历史位姿，仍需检查二维图与点云包是否对齐。
地图包处理、描述子生成和先验定位准备流程见 `src/z-localization/README.md`。

## 导航

```bash
ros2 launch rm_nav_bringup bringup_real.launch.py \
  mode:=nav world:=site \
  prior_map:="$PWD/maps/site_prior" \
  map:="$PWD/maps/site.yaml"
```

`prior_map` 必须是 Z-Localization 地图包目录，不是旧 ICP 使用的单个 `.pcd`。
默认路径为包内 `PCD/<world>/`，二维图默认是 `map/<world>.yaml`；两者必须使用一致的坐标系。
`nav.sh` 和 `fake.sh` 已切换算法名称，但仍使用原来的 `world:=footprint`，使用前需准备对应地图包。

其他参数：

- `driver:=false`：已有驱动或回放 bag 时关闭内置驱动；输入须为 Livox CustomMsg。
- `use_sim_time:=true`：配合 `ros2 bag play ... --clock`。
- `driver_config:=/absolute/MID360_config.json`：指定本机雷达网络配置。
- `frontend_config:=...`、`backend_config:=...`：替换算法配置文件。
- `use_uwb:=true`：启用 UWB；默认关闭，启用前需配置本车天线外参及输入。
- `params_file:=...`：替换 Nav2 参数；保留虚拟速度入口使用的 `SENTRY_NAV_PARAMS_FILE` 兼容方式。
- `lio_rviz:=true`：打开后端 RViz；`nav_rviz:=false`：关闭导航 RViz。

后端运行配置通过临时继承文件覆盖 TF、车体外参、UWB 开关和地图保存目录；退出时删除临时文件，源配置不被改写。

## 离线验证

```bash
source /opt/ros/humble/setup.bash
python3 -m pytest -q src/rm_nav_bringup/test/test_zlio_integration.py
```

这些测试不启动雷达或机器人，不能替代实机 TF、地图对齐和导航联调。
