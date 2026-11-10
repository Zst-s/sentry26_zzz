#!/usr/bin/env bash

WORKSPACE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 地图配置：修改这三项即可。先验地图必须是 Z-Localization 地图包目录。
WORLD="${WORLD:-footprint}"
PRIOR_MAP_DIR="${PRIOR_MAP_DIR:-$WORKSPACE_DIR/src/maps/$WORLD}"
GRID_MAP_YAML="${GRID_MAP_YAML:-$WORKSPACE_DIR/src/rm_nav_bringup/map/$WORLD.yaml}"

# 巡航启动开关：true 随导航启动；false 不启动巡航节点。修改后重启导航生效。
CRUISE="${CRUISE:-false}"
case "$CRUISE" in
    true|false) ;;
    *) echo "CRUISE 必须为 true 或 false" >&2; exit 1 ;;
esac
ENABLE_FAKE_VELOCITY="${ENABLE_FAKE_VELOCITY:-true}"
case "$ENABLE_FAKE_VELOCITY" in
    true|false) ;;
    *) echo "ENABLE_FAKE_VELOCITY 必须为 true 或 false" >&2; exit 1 ;;
esac

# 将路径转义后传入新终端，支持路径包含空格。
printf -v WORLD_ARG '%q' "world:=$WORLD"
printf -v PRIOR_MAP_ARG '%q' "prior_map:=$PRIOR_MAP_DIR"
printf -v GRID_MAP_ARG '%q' "map:=$GRID_MAP_YAML"
printf -v CRUISE_ARG '%q' "cruise:=$CRUISE"
printf -v FAKE_VELOCITY_ARG '%q' "enable_fake_velocity:=$ENABLE_FAKE_VELOCITY"
printf -v WORKSPACE_QUOTED '%q' "$WORKSPACE_DIR"
printf -v DECISION_CONFIG_QUOTED '%q' "$WORKSPACE_DIR/src/decision/config/trans_node4.yaml"

#colcon build
echo "1" | sudo -S sudo chmod 666 /dev/ttyACM0
# 1

cmds=(  "ros2 launch rm_nav_bringup bringup_real.launch.py \
            $WORLD_ARG \
            mode:=nav \
            $CRUISE_ARG \
            $PRIOR_MAP_ARG \
            $GRID_MAP_ARG \
            $FAKE_VELOCITY_ARG \
            lio:=zlio \
            localization:=zlio \
            nav_rviz:=True"
        "ros2 run decision referee_map_tf"
        "ros2 run decision trans_node4 --ros-args \
        --params-file $DECISION_CONFIG_QUOTED"
        "ros2 run decision rosout_node"
        #"ros2 run decision decision"
        #"ros2 run decision mytest"
        #"ros2 run py_tools publish_initial_pose 0.0 0.0 0.0 0.0 0.0 0.0 0.0"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd -- $WORKSPACE_QUOTED && source install/setup.bash && $cmd;exec bash;"
	sleep 0.2
done

sleep 12
echo Current CMD : "ros2 launch rm_bt bt_tree.launch.py"
gnome-terminal -- bash -c "cd -- $WORKSPACE_QUOTED && source install/setup.bash && ros2 launch rm_bt bt_tree.launch.py;exec bash;"
sleep 0.2
