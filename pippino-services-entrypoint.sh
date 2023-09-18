#!/usr/bin/env bash

# export ROS_MASTER_URI=http://192.168.0.32:11311
export ROS_DISTRO=humble

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/pippino_ws/src/cyclonedds.xml 

[ -f /opt/ros/$ROS_DISTRO/setup.bash ] && source /opt/ros/$ROS_DISTRO/setup.bash || source /opt/ros/$ROS_DISTRO/install/setup.bash
source /pippino_ws/install/local_setup.bash
source /video_ws/install/local_setup.bash
source /services_ws/install/local_setup.bash
# source /nav2_ws/install/local_setup.bash
ros2 launch pippino_bringup pippino.launch.py navigation:=false
#bash

# echo $CYCLONEDDS_URI

