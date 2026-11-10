#!/usr/bin/env bash
# Z-Localization 实机建图；可用 ./mapping.sh c9 指定地图名。
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
world="${1:-${WORLD:-c8}}"
if (( $# > 1 )) || [[ ! "$world" =~ ^[a-zA-Z0-9_-]+$ ]]; then
    echo '用法：./mapping.sh [地图名]；地图名仅允许字母、数字、下划线和连字符。' >&2
    exit 2
fi
if ! command -v gnome-terminal >/dev/null 2>&1; then
    echo '未找到 gnome-terminal，请先安装终端程序。' >&2
    exit 1
fi
gnome-terminal -- bash -c '
    run_mapping() {
        cd -- "$1" || return
        source /opt/ros/humble/setup.bash || return
        source install/setup.bash || return
        export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"
        export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"
        printf "建图名称：%s\n后端保存目录：%s\n" "$2" "$1/maps/${2}_raw"
        ros2 launch rm_nav_bringup bringup_real.launch.py \
            mode:=mapping world:="$2" lio:=zlio localization:=zlio \
            lio_rviz:=true nav_rviz:=true cruise:=false \
            save_directory:="$1/maps/${2}_raw"
    }
    run_mapping "$@"
    result=$?
    printf "\n建图进程已退出，退出码：%s。终端保留供查看日志。\n" "$result"
    exec bash
' bash "$workspace_dir" "$world"
