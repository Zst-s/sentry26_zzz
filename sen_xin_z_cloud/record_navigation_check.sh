#!/usr/bin/env bash
# Record current sen_xin_z_cloud navigation diagnostics without starting the robot.
set -eo pipefail

workspace_dir=${SEN_XIN_Z_CLOUD_DIR:-/home/newmaker-12/sen_xin_z_cloud_20261107/sen_xin_z_cloud}
setup_file="$workspace_dir/install/setup.bash"
if [[ ! -f "$setup_file" ]]; then
  echo "找不到工作空间安装文件: $setup_file；请先构建 sen_xin_z_cloud" >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$setup_file"
export ROS_LOCALHOST_ONLY=0

if (( $# > 1 )); then
  echo "用法: $0 [录包输出目录]" >&2
  exit 2
fi
output_dir=${1:-"${HOME}/navigation_check_$(date +%Y%m%d_%H%M%S)"}
if [[ -z "$output_dir" || -e "$output_dir" ]]; then
  echo "输出目录为空或已存在，请换一个新目录: $output_dir" >&2
  exit 2
fi

echo "录包输出目录: $output_dir"
echo "请先录包再发 Goal；用 Ctrl+C 结束并等待保存完成。"
echo "录制期间不要读取正在写入的 .db3 文件。"
exec ros2 bag record --include-hidden-topics -o "$output_dir" \
  /livox/lidar \
  /livox/imu \
  /frontend/cloud_deskewed \
  /frontend/frame \
  /frontend/odometry \
  /frontend/session \
  /Odometry \
  /lidar_odometry \
  /base_imu \
  /localization/odometry \
  /backend_node/localization_state \
  /backend_node/localization_pose \
  /scan \
  /segmentation/ground \
  /segmentation/obstacle \
  /terrain/grid \
  /terrain/passage/structure \
  /rog_map/raw_occupied \
  /map \
  /tf \
  /tf_static \
  /initialpose \
  /goal_pose \
  /navigation_goals \
  /plan \
  /local_plan \
  /teb_poses \
  /teb_markers \
  /cmd_vel_nav \
  /cmd_vel \
  /cruise_speed \
  /find_already \
  /lose_already \
  /enemy_pose \
  /nav_ready \
  /target_stance \
  /robot_status \
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
