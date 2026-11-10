#!/usr/bin/env bash
# 停止 ZLIO 接收传感器输入；等待末尾地图更新完成后，再运行 save_pcd.sh。
set -e

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    source "$WORKSPACE_DIR/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "未找到 ros2，请先加载 ROS 2 环境。" >&2
    exit 1
fi

ros2 service call /stop std_srvs/srv/Trigger "{}"
