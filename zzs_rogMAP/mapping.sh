#colcon build 
cmds=(  "ros2 launch rm_nav_bringup bringup_real.launch.py \
            world:=my_map9 \
            mode:=mapping  \
            lio:=fastlio \
            lio_rviz:=False \
            nav_rviz:=True \
            perception:=rog_map"
        #"ros2 run decision trans_node"
        # "ros2 run decision decision"
        # "ros2 bag record /livox/imu /livox/lidar /livox/lidar/pointcloud"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.2
done
