#!/bin/bash

docker stop pippino_realsense
docker rm pippino_realsense

ROS_DISTRO=humble

docker run -d --rm \
     -v /dev:/dev \
     --name pippino_realsense \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     pippino/ros2:realsense-$ROS_DISTRO
     # --restart always \


