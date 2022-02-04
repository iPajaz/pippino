#!/usr/bin/env bash

distro=galactic
source /opt/ros/$distro/install/setup.bash
source /realsense_ws/install/local_setup.bash
ros2 launch realsense2_camera rs_launch.py config_file:=\'/pippino_ws/src/d455.yaml\'
# bash