# 先手动调用 /stop 并等待末尾地图更新完成，再运行本脚本。
ros2 service call /backend_node/save_map std_srvs/srv/Trigger "{}"
