#!/bin/bash

docker stop pippino_realsense
docker rm pippino_realsense
docker stop pippino_realsense_t265
docker rm pippino_realsense_t265
docker stop pippino_realsense_d455
docker rm pippino_realsense_d455

# ROS_DISTRO=humble

docker run -d --rm \
     -v /dev:/dev \
     --name pippino_realsense_d455 \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     -v /dev/shm:/dev/shm \
     --entrypoint=/pippino_ws/src/pippino-realsense_d455-entrypoint.sh \
     pippino/ros2:realsense-rsusb-$ROS_DISTRO
     # -e "RMW_IMPLEMENTATION" \
     # --restart always \


docker run -d --rm \
     -v /dev:/dev \
     --name pippino_realsense_t265 \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     -v /dev/shm:/dev/shm \
     --entrypoint=/pippino_ws/src/pippino-realsense_t265-entrypoint.sh \
     pippino/ros2:realsense-rsusb-$ROS_DISTRO
     # -e "RMW_IMPLEMENTATION" \
     # --restart always \


