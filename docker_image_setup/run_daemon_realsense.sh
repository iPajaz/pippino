#!/bin/bash

docker stop pippino_realsense
docker rm pippino_realsense

distro=galactic

docker run -it --rm \
     -v /dev:/dev \
     --name pippino_realsense \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     pippino/ros2:realsense-$distro
     # --restart always \


