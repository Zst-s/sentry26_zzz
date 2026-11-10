#!/usr/bin/env bash
set -eo pipefail
WORKSPACE_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
WORLD=${WORLD:-footprint}
case "${1:-}" in
    --help|-h)
        echo '用法：./nav.sh [地图名] [name:=value ...]'
        echo '示例：./nav.sh c8；./bt.sh c8；WORLD=c8 ./nav.sh'
        echo '二维地图：src/rm_nav_bringup/map/<地图名>.yaml'
        echo '定位地图包：src/maps/<地图名>（可通过 PRIOR_MAP_DIR 指定）'
        exit 0 ;;
esac
if [[ $# -gt 0 && "$1" != *":="* && "$1" != --* ]]; then
    WORLD=$1
    shift
fi
launch_args=()
for arg in "$@"; do
    case "$arg" in
        world:=*) WORLD=${arg#world:=} ;;
        *) launch_args+=("$arg") ;;
    esac
done
if [[ -z "${PRIOR_MAP_DIR:-}" ]]; then
    PRIOR_MAP_DIR="$WORKSPACE_DIR/src/maps/$WORLD"
    for candidate in "$WORKSPACE_DIR/src/maps/$WORLD" \
                     "$WORKSPACE_DIR/src/rm_nav_bringup/map/$WORLD" \
                     "$WORKSPACE_DIR/src/rm_nav_bringup/PCD/$WORLD" \
                     "$WORKSPACE_DIR/src/rm_nav_bringup/pcd/$WORLD"; do
        if [[ -d "$candidate" && -f "$candidate/map.pcd" && -f "$candidate/keyframes.txt" ]]; then
            PRIOR_MAP_DIR=$candidate
            break
        fi
    done
fi
GRID_MAP_YAML=${GRID_MAP_YAML:-$WORKSPACE_DIR/src/rm_nav_bringup/map/$WORLD.yaml}
SERIAL_PORT=${SERIAL_PORT:-/dev/ttyACM0}
ENABLE_SERIAL=${ENABLE_SERIAL:-true}
ENABLE_FAKE_VELOCITY=${ENABLE_FAKE_VELOCITY:-true}
CRUISE=${CRUISE:-false}
for setting in "$CRUISE" "$ENABLE_FAKE_VELOCITY"; do
    case "$setting" in
        true|false) ;;
        *) echo "CRUISE 和 ENABLE_FAKE_VELOCITY 必须为 true 或 false" >&2; exit 1 ;;
    esac
done
cd -- "$WORKSPACE_DIR"
if [[ ! -f "$GRID_MAP_YAML" ]]; then
    echo "二维地图不存在：$GRID_MAP_YAML" >&2; exit 1
fi
if [[ ! -d "$PRIOR_MAP_DIR" ]]; then
    echo "Z-Localization定位地图包目录不存在：$PRIOR_MAP_DIR" >&2
    echo '二维yaml/pgm不能代替定位地图包；请通过PRIOR_MAP_DIR指定配套地图目录。' >&2; exit 1
fi
printf '地图：%s\n二维地图：%s\n定位地图包：%s\n' "$WORLD" "$GRID_MAP_YAML" "$PRIOR_MAP_DIR"
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
if [[ ! -f install/setup.bash ]]; then
    echo '请先在工作区运行 colcon build --symlink-install。' >&2; exit 1
fi
source install/setup.bash
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"
if [[ "$ENABLE_SERIAL" == true && (! -r "$SERIAL_PORT" || ! -w "$SERIAL_PORT") ]]; then
    echo "串口不存在或当前用户无读写权限：$SERIAL_PORT" >&2; exit 1
fi
exec ros2 launch decision passage_navigation.launch.py \
    "world:=$WORLD" "prior_map:=$PRIOR_MAP_DIR" "map:=$GRID_MAP_YAML" \
    "serial_port:=$SERIAL_PORT" "enable_serial:=$ENABLE_SERIAL" \
    "enable_fake_velocity:=$ENABLE_FAKE_VELOCITY" \
    "cruise:=$CRUISE" \
    "nav_rviz:=${NAV_RVIZ:-true}" "${launch_args[@]}"
