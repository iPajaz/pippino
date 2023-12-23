#!/bin/bash

#export ROS_DISTRO=humble

#export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#export FASTRTPS_DEFAULT_PROFILES_FILE=~/pippino_ws/src/fastrtps_client.xml

. /opt/ros/$ROS_DISTRO/install/setup.bash
. ~/rviz_ws/install/local_setup.bash
. ~/services_ws/install/local_setup.bash
. ~/realsense_ws/install/local_setup.bash
. ~/pippino_ws/install/local_setup.bash
. ~/plugin_dev_ws/install/local_setup.bash
. ~/autodock_ws/install/local_setup.bash
. ~/navigation_humble_ws/install/local_setup.bash

# ros2 daemon stop
# ros2 daemon start
