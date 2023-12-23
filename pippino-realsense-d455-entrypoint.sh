#!/usr/bin/env bash

source /opt/ros/$ROS_DISTRO/install/setup.bash
source /home/michele/realsense_ws/install/local_setup.bash
source /home/michele/pippino_ws/install/local_setup.bash

ros2 launch pippino_bringup rs_d455_launch.py
