#!/bin/bash
ROS_DISTRO=humble

docker run -it --rm \
     --name pippino_realsense \
     -v /home/michele/pippino_ws:/pippino_ws \
     --entrypoint=/pippino_ws/src/test_rtps_entrypoint.sh \
     --net=host \
     -v /dev/shm:/dev/shm \
     --env CONTAINER=realsense \
     pippino/ros2:realsense-$ROS_DISTRO

