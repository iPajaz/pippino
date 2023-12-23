#!/usr/bin/env bash
# export ROS_DISTRO=humble

# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# export FASTRTPS_DEFAULT_PROFILES_FILE=/home/michele/pippino_ws/src/fastrtps_client.xml


[ -f /opt/ros/$ROS_DISTRO/setup.bash ] && source /opt/ros/$ROS_DISTRO/setup.bash || source /opt/ros/$ROS_DISTRO/install/setup.bash
source /home/michele/pippino_ws/install/local_setup.bash
source /services_ws/install/local_setup.bash
# ros2 launch pippino_bringup pippino.launch.py
bash

