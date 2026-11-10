# TEB 内折诊断

只观察优化候选，不改变轨迹约束、速度输出或碰撞检查。默认关闭。
参数位于 controller_server 节点根参数下，不在 FollowPath 下。

## 开关

编译、安装并重启导航后：

```bash
ros2 param set /controller_server teb_diagnostics.enabled true
ros2 param get /controller_server teb_diagnostics.enabled
```

关闭：

```bash
ros2 param set /controller_server teb_diagnostics.enabled false
```

每个优化器实例最多每秒采样两次完整优化调用；每次包含 input 和各个外循环的 before/after。
不是每秒只有两条消息。开启会产生额外开销，复现时禁用底盘运动，采集短时间后关闭。
现阶段建议保持单轨迹规划（enable_homotopy_class_planning=false），多个候选会分别产生数据。

## 显示和记录

RViz 添加 MarkerArray，话题 /teb_diagnostics/markers，Reliability 选 Best Effort。
命名空间按“优化器实例地址/阶段/外循环编号”区分，可在 RViz 中单独勾选。

- input：蓝色，更新/裁剪或初始化后、autoResize 前的轨迹，不是原始全局路径。
- before：红色，本次外循环调整采样点并建图后、优化前。
- after：绿色，本次外循环优化后。
- failed：优化失败时的候选，不表示可执行。
- 黄色关联线：实际 EdgeObstacle/EdgeInflatedObstacle 的轨迹点到障碍物质心。
- 紫色关联线：实际引导点关联。
- 白点：输入障碍物的质心；对于线段/多边形，质心不是完整形状。

标记 0.6 秒过期。不同阶段和外循环会叠加，比较时只打开需要的两个命名空间。
同一次采样用 details 的 instance+cycle 对齐；不要把 RViz 混合显示当成单条路径。

```bash
ros2 bag record /teb_diagnostics/markers /teb_diagnostics/details /plan /local_plan /local_costmap/costmap /local_costmap/costmap_updates /tf /tf_static
```

单独查看文本：

```bash
ros2 topic echo /teb_diagnostics/details --qos-reliability best_effort --full-length
```

details 包含：阶段、时间、初始化方式、各姿态 x/y/yaw 与 dt、参考路径（reference_pose）、
引导点输入、实际关联及模型轮廓距离、各优化边类型的 weighted_chi2（信息矩阵加权平方误差）。
类型名称可能包含 C++ 编译器编码，但保留 EdgeObstacle 等可识别名称。
各外循环可能采用不同自适应权重，只宜直接比较同一外循环的 before/after 代价。
障碍关联线当前仅显示静态障碍边；动态障碍边仍计入类型代价统计。

所有发布数据都在最终可行性检查之前，不代表轨迹安全，也不含原始 costmap 格子的真实代价值。
关联距离是 TEB 机器人模型计算的距离，不等价于最终 costmap footprint 碰撞检查结果。

## 对照方法

保持其他配置不变，分别测试全局膨胀半径 0.43 和 0.38，重新开始同一目标。
重点检查第一次发生内折的 cycle：before 是否已内折，after 是否更严重，内折点是否关联了
对应障碍，引导点对应位置是否回跳，以及 yaw 是否先出现异常变化。
warm_start 只表示复用轨迹，不等于它是根因。不要为了让诊断“正常”而关闭碰撞检查。
