# 虚拟速度转换

虚拟速度转换现已整合进 `bringup_real.launch.py`。`nav.sh` 默认通过
`ENABLE_FAKE_VELOCITY=true` 启动；设为 `false` 可关闭。直接启动时使用：

```bash
ros2 launch rm_nav_bringup bringup_real.launch.py enable_fake_velocity:=true
```

启用后，Nav2 以 `base_link_fake` 为车体坐标系规划平移。导航控制器、速度平滑器和恢复行为把速度送到 `/cmd_vel_nav`；
`fake_vel_transform.py` 根据 `odom → base_link` 的当前姿态换算后，向 `/cmd_vel` 发布实际底盘坐标系速度。
它还发布 `odom → base_link_fake`，其平移跟随车体，姿态保持与 `odom` 对齐。
控制器的速度反馈从 `/base_imu` 转换到虚拟坐标系后发布为 `/base_imu_fake`；关闭虚拟模式时控制器直接读取 `/base_imu`。两者均来自 ZLIO 的车体里程计估计。
实机参数从当前 Nav2 YAML 生成临时覆盖文件，退出启动后清理。

该模式要求底盘能执行 `cmd_vel.linear.y`。在窄通道中，应按车体实际旋转范围核对 footprint。
