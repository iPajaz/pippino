!/usr/bin/env bash

# export ROS_MASTER_URI=http://192.168.0.32:11311
# export ROS_DISTRO=humble

# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/michele/pippino_ws/src/fastrtps_client.xml

# function ros_source_env() 
# {
# 	if [ -f "$1" ]; then
# 		echo "sourcing   $1"
# 		source "$1"
# 	else
# 		echo "notfound   $1"
# 	fi	
# }

# ros_source_env "$ROS_ROOT/install/setup.bash"
# ros_source_env "/ros_deep_learning/install/setup.bash"

# if [ -d "/ros_deep_learning/install/ros_deep_learning" ]; then
# 	export AMENT_PREFIX_PATH="/ros_deep_learning/install/ros_deep_learning:$AMENT_PREFIX_PATH"
# fi

echo "alias run=\"ros2 launch ros_deep_learning detectnet.ros2.launch\"" >> /root/.bashrc

bash