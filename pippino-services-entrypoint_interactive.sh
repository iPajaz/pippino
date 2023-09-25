#!/usr/bin/env bash
export ROS_DISTRO=humble

# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# export CYCLONEDDS_URI=/pippino_ws/src/cyclonedds.xml 
export ROS_DOMAIN_ID=0

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/pippino_ws/src/fastrtps.xml

[ -f /opt/ros/$ROS_DISTRO/setup.bash ] && source /opt/ros/$ROS_DISTRO/setup.bash || source /opt/ros/$ROS_DISTRO/install/setup.bash
source /pippino_ws/install/local_setup.bash
source /services_ws/install/local_setup.bash
# ros2 launch pippino_bringup pippino.launch.py
bash
