#!/bin/bash

docker stop pippino_realsense_d455
docker rm pippino_realsense_d455

docker run -it --rm \
     -v /dev:/dev \
     --name pippino_realsense_d455 \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/home/michele/pippino_ws \
     --net=host \
     -v /dev/shm:/dev/shm \
     --entrypoint="/bin/bash" \
     -e "ROS_DISTRO" \
     -e "ROS_DOMAIN_ID" \
     -e "ROS_DISCOVERY_SERVER" \
     pippino/ros2:realsense-rsusb-d455-$ROS_DISTRO
     # -e "RMW_IMPLEMENTATION" \
     # -e "FASTRTPS_DEFAULT_PROFILES_FILE" \

     # -u $(id -u):$(id -g) \
# . /pippino_ws/install/local_setup.bash ;. /realsense_ws/install/local_setup.bash
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
# ros2 launch realsense2_camera rs_launch.py 
# ros2 launch pippino_bringup pippino.launch.py
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true
