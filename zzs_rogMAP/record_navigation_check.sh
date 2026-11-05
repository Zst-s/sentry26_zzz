#!/usr/bin/env bash
# Record navigation diagnostics without launching navigation or sending commands.
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/newmaker-12/zzs_rogMAP/install/setup.bash
export ROS_LOCALHOST_ONLY=0
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"

if (( $# > 1 )); then
  echo "用法: $0 [录包输出目录]" >&2
  exit 2
fi
output_dir=${1:-"${HOME}/passage_check_$(date +%Y%m%d_%H%M%S)"}
if [[ -z "$output_dir" || -e "$output_dir" ]]; then
  echo "输出目录为空或已存在，请换一个新目录: $output_dir" >&2
  exit 2
fi

echo "录包输出目录: $output_dir"
echo "请先录包再发Goal；用Ctrl+C结束并等待保存完成。"
echo "录制期间不要读取正在写入的.db3文件。"
exec ros2 bag record --include-hidden-topics -o "$output_dir" \
  /livox/lidar \
  /livox/lidar/pointcloud \
  /livox/imu \
  /imu/data \
  /cloud_registered \
  /cloud_registered_dense_terrain \
  /scan \
  /tf \
  /tf_static \
  /Odometry \
  /navigation/odometry \
  /pointlio/status \
  /passage/localization_motion_ready \
  /initialpose \
  /goal_pose \
  /clicked_point \
  /passage/desired_goal \
  /plan \
  /local_plan \
  /teb_poses \
  /teb_markers \
  /cmd_vel_nav \
  /passage/nav_cmd_vel \
  /passage/serial_tx_status \
  /passage/chassis_cmd_vel \
  /nav_ready \
  /target_stance \
  /map \
  /passage/static_corrected_map \
  /terrain/grid \
  /passage/mapping_terrain \
  /rog_map/raw_occupied \
  /terrain/passage/structure \
  /passage/verified_static_clearing \
  /passage/static_correction_source \
  /passage/memory_status \
  /global_costmap/costmap \
  /global_costmap/costmap_raw \
  /local_costmap/costmap \
  /local_costmap/costmap_raw \
  /local_costmap/published_footprint \
  /navigate_to_pose/_action/status \
  /navigate_to_pose/_action/feedback \
  /follow_path/_action/status \
  /follow_path/_action/feedback \
  /rosout
