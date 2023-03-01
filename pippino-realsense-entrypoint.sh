#!/usr/bin/env bash

distro=galactic

#source /opt/ros/$distro/install/setup.bash

source /opt/ros/galactic/setup.bash
source /realsense_ws/install/local_setup.bash
source /pippino_ws/install/local_setup.bash
# ros2 launch realsense2_camera rs_launch.py config_file:=\'/pippino_ws/src/d455.yaml\'
# bash

ros2 launch pippino_bringup rs_d400_and_t265_launch.py
