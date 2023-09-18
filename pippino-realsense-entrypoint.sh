#!/usr/bin/env bash


# export ROS_MASTER_URI=http://192.168.0.32:11311
export ROS_DISTRO=humble

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=/pippino_ws/src/cyclonedds.xml 

[ -f /opt/ros/$ROS_DISTRO/setup.bash ] && source /opt/ros/$ROS_DISTRO/setup.bash || source /opt/ros/$ROS_DISTRO/install/setup.bash
source /realsense_ws/install/local_setup.bash
source /pippino_ws/install/local_setup.bash

# ros2 launch realsense2_camera rs_launch.py config_file:=\'/pippino_ws/src/d455.yaml\'
# bash

ros2 launch pippino_bringup rs_d400_and_t265_launch.py
