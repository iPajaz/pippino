#!/usr/bin/env bash

ROS_DISTRO="foxy"
source /opt/ros/$ROS_DISTRO/setup.bash
source /pippino_ws/install/local_setup.bash
source /services_ws/install/local_setup.bash
ros2 launch pippino_bringup pippino.launch.py
# exec bash -c "trap \"pkill rplidar\" HUP INT QUIT TERM; sleep infinity & wait"

