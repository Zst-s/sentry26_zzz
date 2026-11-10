#!/usr/bin/env bash
# 将全局 PCD 转换为 Z-Localization 先验地图包。
set -e

on_exit() {
    local result=$?
    if (( result != 0 )); then
        printf '\n地图转换失败，退出码：%s。请保留上方完整报错。\n' "$result" >&2
        if [[ -t 0 && -t 2 ]]; then
            read -r -p '按回车关闭窗口…' _ || true
        fi
    fi
    return "$result"
}
trap on_exit EXIT

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 输入、输出路径；输出目录与 nav.sh 的默认先验地图目录一致。
# 输出目录必须不存在或为空，工具不会直接覆盖已有地图包。
WORLD="${WORLD:-c8}"
INPUT_PCD="${INPUT_PCD:-$WORKSPACE_DIR/src/rm_nav_bringup/PCD/${WORLD}.pcd}"
OUTPUT_DIR="${OUTPUT_DIR:-$WORKSPACE_DIR/src/maps/$WORLD}"

# 以下范围和高度是示例，必须按 PCD 坐标和实际安装修改，单位均为米。
# X/Y 是机器人可能行驶区域在地图中的坐标范围。
X_MIN="${X_MIN:-0}"
X_MAX="${X_MAX:-28}"
Y_MIN="${Y_MIN:-0}"
Y_MAX="${Y_MAX:-15}"
# SENSOR_Z = 地面在地图中的 Z 坐标 + 传感器离地高度。
# 例如地面 Z=-0.35、离地 0.35 m 时，SENSOR_Z 应为 0。
SENSOR_Z="${SENSOR_Z:-0.066}"
# 描述子使用的离地高度，应与 backend.yaml 的 common.scan_context.lidar_height 一致。
SC_LIDAR_HEIGHT="${SC_LIDAR_HEIGHT:-0.066}"

# 可先使用以下采样/降采样参数；视场参数按水平安装设置。
BUNDLE_STEP="${BUNDLE_STEP:-1.5}"
MAP_VOXEL="${MAP_VOXEL:-0.15}"
SCAN_VOXEL="${SCAN_VOXEL:-0.4}"
FOV_DOWN="${FOV_DOWN:--7}"
FOV_UP="${FOV_UP:-52}"

if [[ -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    source "$WORKSPACE_DIR/install/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "未找到 ros2，请先加载 ROS 2 环境并编译 backend 包。" >&2
    exit 1
fi
if [[ ! -f "$INPUT_PCD" ]]; then
    printf '输入 PCD 不存在：%s\n请修改脚本中的 INPUT_PCD。\n' "$INPUT_PCD" >&2
    exit 1
fi

# 与 mapping.sh 一致，避免外部环境中的旧 libusb 覆盖系统 PCL 依赖。
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"

printf '输入 PCD：%s\n输出地图包：%s\n' "$INPUT_PCD" "$OUTPUT_DIR"
ros2 run backend build_pcd_map_bundle \
    --input "$INPUT_PCD" \
    --out-bundle "$OUTPUT_DIR" \
    --x-min "$X_MIN" --x-max "$X_MAX" \
    --y-min "$Y_MIN" --y-max "$Y_MAX" \
    --sensor-z "$SENSOR_Z" \
    --sc-lidar-height "$SC_LIDAR_HEIGHT" \
    --bundle-step "$BUNDLE_STEP" \
    --map-voxel "$MAP_VOXEL" \
    --scan-voxel "$SCAN_VOXEL" \
    --fov-down "$FOV_DOWN" --fov-up "$FOV_UP" \
    --descriptor-type scan_context
