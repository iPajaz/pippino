#!/usr/bin/env bash


# export ROS_MASTER_URI=http://192.168.0.32:11311
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=1

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/michele/pippino_ws/src/fastrtps.xml

[ -f /opt/ros/$ROS_DISTRO/setup.bash ] && source /opt/ros/$ROS_DISTRO/setup.bash || source /opt/ros/$ROS_DISTRO/install/setup.bash
source /pippino_ws/install/local_setup.bash

# ros2 launch realsense2_camera rs_launch.py config_file:=\'/pippino_ws/src/d455.yaml\'
# bash
(trap 'kill 0' SIGINT; ros2 run demo_nodes_cpp talker __ns:=/$CONTAINER & ros2 run demo_nodes_cpp listener __ns:=/eightball & ros2 run demo_nodes_cpp listener __ns:=/realsense & ros2 run demo_nodes_cpp listener __ns:=/services )
