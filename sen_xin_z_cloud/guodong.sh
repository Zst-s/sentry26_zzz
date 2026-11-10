#!/usr/bin/env bash
# Navigation and serial must run separately.
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
rebuild=false
case "${1:-}" in
  --help|-h)
    echo '用法: ./guodong.sh [--build]'
    echo '启动 guodong.xml；首次编译决策和串口依赖，--build 强制重新编译。'
    echo '按到点顺序执行；导航和串口需另外运行。fold_wait_ms 可通过 GUODONG_FOLD_WAIT_MS 设置。'
    exit 0 ;;
  --build) rebuild=true; shift ;;
  '') ;;
  *) echo '用法: ./guodong.sh [--build]' >&2; exit 2 ;;
esac
if (( $# != 0 )); then echo '参数过多' >&2; exit 2; fi
cd -- "$workspace_dir"
ros_distro=${ROS_DISTRO:-}
if [[ -z "$ros_distro" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then ros_distro=humble
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then ros_distro=jazzy
  else echo '未找到 ROS 环境，请先 source 对应 ROS 的 setup.bash。' >&2; exit 1
  fi
fi
if [[ ! -f "/opt/ros/$ros_distro/setup.bash" ]]; then
  echo "未找到 /opt/ros/$ros_distro/setup.bash" >&2; exit 1
fi
source "/opt/ros/$ros_distro/setup.bash"
if [[ "$rebuild" == true || ! -f install/setup.bash || ! -x install/rm_bt/lib/rm_bt/rm_bt || ! -f install/rm_bt/share/rm_bt/behavior_tree/guodong.xml || ! -x install/decision/lib/decision/trans_node2 || ! -f install/rm_serial_protocol/include/rm_serial_protocol/nav_rx.hpp ]]; then
  echo "编译 rm_bt、decision 及通信依赖（ROS $ros_distro）……"
  colcon build --packages-up-to rm_bt decision --symlink-install
fi
source install/setup.bash
fold_wait_ms=${GUODONG_FOLD_WAIT_MS:-2000}
if [[ ! "$fold_wait_ms" =~ ^[0-9]{1,8}$ ]]; then
  echo 'GUODONG_FOLD_WAIT_MS 必须是非负整数毫秒（最多 8 位）。' >&2; exit 2
fi
printf '启动过洞决策：到第一点 -> 洞口 -> 收头等待 %s ms -> 过洞 -> 抬头。\n' "$fold_wait_ms"
exec ros2 run rm_bt rm_bt --ros-args \
  -p tree_xml_file:=behavior_tree/guodong.xml \
  -p enable_groot:=false \
  -p tick_period_milliseconds:=50 \
  -p stop_on_completion:=true \
  -p fold_wait_ms:="$fold_wait_ms"
