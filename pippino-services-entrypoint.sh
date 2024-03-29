#!/usr/bin/env bash

# export ROS_MASTER_URI=http://192.168.0.32:11311
# export ROS_DISTRO=humble

# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/pippino_ws/src/fastrtps_client.xml


[ -f /opt/ros/$ROS_DISTRO/setup.bash ] && source /opt/ros/$ROS_DISTRO/setup.bash || source /opt/ros/$ROS_DISTRO/install/setup.bash
source /home/michele/pippino_ws/install/local_setup.bash
source /video_ws/install/local_setup.bash
source /services_ws/install/local_setup.bash
# source /nav2_ws/install/local_setup.bash
ros2 launch pippino_bringup pippino.launch.py navigation:=false
#bash

# echo $CYCLONEDDS_URI

