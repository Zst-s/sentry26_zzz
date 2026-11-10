# trans_node4 参数

首次添加参数功能需要编译 decision；之后改 sender_id 不需要编译，只需停止旧节点并重新启动。
务必先禁能电机/使用硬件急停，不能依赖停止串口节点让底盘停车。

编辑本目录 trans_node4.yaml：红方哨兵填 7，蓝方哨兵填 107（十进制），必须匹配裁判系统实际 robot_id。
sender_id 是启动时读取的只读 ROS 参数，不支持运行中 ros2 param set。

## 始终读取源码中的 YAML（修改后不用编译或复制到 install）

```bash
source /home/lyg/sen_xin/install/setup.bash
ros2 launch decision trans_node4.launch.py params_file:=/home/lyg/sen_xin/src/decision/config/trans_node4.yaml
```

也可直接运行：

```bash
ros2 run decision trans_node4 --ros-args --params-file /home/lyg/sen_xin/src/decision/config/trans_node4.yaml
```

不要同时启动两个 trans_node4；如果其他 launch 已启动它，应给原 Node 的 parameters 加入上述 YAML，
不要再启动第二个串口进程。启动日志会打印 sender_id，0x0307 调试输出也包含该值。
不带 params_file 的新 launch 默认读取 install 下的 YAML 副本，编辑 src 文件不会自动改变该副本。
本启动文件仅启动串口节点，不启动 referee_map 静态 TF、导航或决策树；这些仍沿用原启动方式。
