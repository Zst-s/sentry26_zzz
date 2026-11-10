# 指定障碍物关联诊断

## 全量模式（默认）

`teb_diagnostics.watch_all` 默认 true，但总开关 `teb_diagnostics.enabled` 仍默认 false。
开启诊断后，input 阶段列出进入优化器的全部障碍物（obstacle_input），每次建图记录所有中间位姿与所有输入障碍物的关联或排除原因。可动态切换：

```bash
ros2 param set /controller_server teb_diagnostics.watch_all true
```

设置 false 恢复下述指定坐标模式。全量模式依然输出指定坐标的存在性检查。

`obstacle_index` 只在当前周期内有效，不能当作跨周期稳定 ID；比较前后周期时应按坐标和几何类型匹配。全量检查指所有进入优化器的障碍物，不会枚举整张地图的空格，也不能看到提取前已被排除的点。若需区分传感器丢点、栅格清除或提取过滤，仍需同步原始数据与 costmap。

日志规模随轨迹点数乘以障碍数增加，可能增加 CPU 和记录负担，并影响故障复现。只在短时排查中开启，录完关闭 enabled；不是每个控制周期都记录。legacy 模式仍只提供输入清单和原有 obstacle_link，不提供新关联策略的逐点排除原因。

沿用 `/controller_server` 的 `teb_diagnostics.enabled` 开关，沿用 `/teb_diagnostics/details` 文本话题。关闭诊断时不逐点收集筛选原因，不改变障碍关联算法。

默认关注 costmap/TEB 全局坐标系中的两个坐标 `(-0.17, 1.55)`、`(-0.15, 1.55)`，每个匹配半径为 0.015m，以障碍质心匹配，不表示障碍物大小。若地图坐标系、定位或地图更换，必须更新坐标。多边形转换器的质心不一定在边缘点位置。

```bash
ros2 param set /controller_server teb_diagnostics.enabled true
ros2 param set /controller_server teb_diagnostics.watch_xy '[-0.17, 1.55, -0.15, 1.55]'
ros2 param set /controller_server teb_diagnostics.watch_radius 0.015
ros2 topic echo /teb_diagnostics/details std_msgs/msg/String --qos-reliability best_effort --full-length > /tmp/teb_obstacle_watch.txt
```

这些参数在规划器初始化时声明。重启导航加载新库后才可设置。保持原有约 500ms 采样间隔，并非每次控制循环都记录；单个采样周期含 input 和各外层优化前后记录。

- `watch_input target=0 ... extracted=1 matches=1`：优化器收到的障碍容器里存在该坐标附近的质心。
- `extracted=0`：容器里没有匹配项。不能据此断言传感器或地图中没有障碍；可能已在提取或后方过滤环节排除，或其质心移动出匹配区域。`nearest_centroid_distance` 有助于区分移动和远离。
- `watch_candidate`：实际匹配到的质心坐标。
- `watch_build`：实际强制关联和截断距离，距离按配置车身模型计算。
- `watch_association ... edge_added=1 reason=force_inclusion`：距离较近，强制建立约束。
- `reason=nearest_on_side`：在普通关联范围内，为该侧最近障碍。
- `edge_added=0 reason=beyond_cutoff`：距离超过关联截断阈值。
- `reason=not_nearest_on_side`：范围内但不是该侧最近障碍。
- `reason=handled_by_dynamic_edges`：交给动态障碍分支，不是简单忽略。
- `reason=first_pose_edge_skipped`：首位姿不建立普通障碍边；尾位姿也不建立。这不是关闭最终轨迹碰撞检查。

`footprint_distance_at_build` 和原因是建图时保存的真实决策，after 阶段沿用相同快照；不是用优化后的位姿反推。原有 `obstacle_link` 中的距离则基于当前阶段位姿，二者可以不同。

当前新关联策略提供逐位姿原因；legacy 策略明确输出 `legacy_mode=1`，不套用新策略的原因。原有障碍约束及其他诊断仍保留。

重点对比同一质心：若消失周期 extracted=0，先查输入/提取链；若 extracted=1 但 edge_added=0，则查看明确筛选原因。仅凭日志仍不能证明 costmap 某格是否清除，需要同步地图记录进一步判断。
