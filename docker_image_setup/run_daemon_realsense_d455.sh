#!/bin/bash

docker stop pippino_realsense_d455
docker rm pippino_realsense_d455

docker run -d --rm \
     -v /dev:/dev \
     --name pippino_realsense_d455 \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/home/michele/pippino_ws \
     --net=host \
     -v /dev/shm:/dev/shm \
     -e "ROS_DISTRO" \
     -e "ROS_DOMAIN_ID" \
     -e "ROS_DISCOVERY_SERVER" \
     --entrypoint=/home/michele/pippino_ws/src/pippino-realsense-d455-entrypoint.sh \
     pippino/ros2:realsense-rsusb-d455-$ROS_DISTRO
     # -e "RMW_IMPLEMENTATION" \
     # --restart always \
