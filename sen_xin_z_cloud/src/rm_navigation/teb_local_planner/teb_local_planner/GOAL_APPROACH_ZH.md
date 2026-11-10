# 末端按距离限速

实现截图中的分段线速度上限：

`v_limit = goal_approach_max_speed * min(1, d / goal_approach_distance)`

其中 d 为当前底盘位置到完整输入全局路径终点的平面欧氏距离，统一转换到局部 costmap 的坐标系；不是到截取后的局部子目标的距离。对于多段任务，这里指当前 FollowPath 收到的路径终点。

在 FollowPath 下配置（默认关闭，实车配置也保持关闭）：

```yaml
goal_approach_enabled: false
goal_approach_distance: 0.5
goal_approach_max_speed: 0.5
```

两项数值参数必须是有限、严格为正的浮点数。支持动态更新：

```bash
ros2 param set /controller_server FollowPath.goal_approach_enabled true
ros2 param set /controller_server FollowPath.goal_approach_enabled false
```

这是位于原有速度饱和之后、阿克曼转向角转换之前的输出限幅，不是新增 g2o 优化边。优化轨迹与诊断 details 不会因为限幅直接改变，效果应查看实际输出速度。原有各轴速度限制、碰撞检查和到达判定均保留。

以 hypot(vx, vy) 计算合成速度，只有超过上限时才按同一比例缩放 vx、vy、omega，保留平移方向和瞬时运动曲率。纯原地旋转不受限制。不会把原本很小的速度提高，也不设置最低速度，避免临近目标强行推进。角速度同比缩放是对截图纯线速度公式的配套处理，并非独立的终端朝向控制。

注意：这是一条距离限速曲线，不保证加速度或加加速度连续，也不能代替下位机的制动能力限制。启用/改参时应停车；目标更新、定位跳变或限速突变可能使指令发生突变。先在安全空地低速验证。若已低速蠕动，该功能可能使其更慢，不能修复轨迹摆动或底盘不执行旋转的问题。

离目标 0.5m 及更远时上限为 0.5m/s，0.25m 时为 0.25m/s，0.1m 时为 0.1m/s。到达仍由原有 goal checker 决定，不需要等待距离精确为零。欧氏距离在需绕障到达目标的场景也会触发减速，这是截图公式本身的性质。

本地验证：goal_approach_test 覆盖区域外/边界/区域内/零距离、横移倒车、低速不提速、纯旋转和非法输入；真实底盘尚未验证。
