#!/usr/bin/env bash
# 在 mapping 模式运行期间保存 ZLIO 先验包、SLAM posegraph 和已验证清障二维图。
set -euo pipefail
if [[ $# -ne 1 || ! "$1" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    echo '用法: ./save_mapping_map.sh 地图名（须与 mapping.sh 的 WORLD 一致）' >&2
    exit 2
fi
WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    echo '请先编译当前工作空间。' >&2
    exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE_DIR/install/setup.bash"
set -u
exec ros2 run rm_nav_bringup export_mapping_map.py "$WORKSPACE_DIR" "$1"
