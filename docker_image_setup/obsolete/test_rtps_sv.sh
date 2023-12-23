#!/bin/bash
ROS_DISTRO=humble

docker run -it --rm \
     --name pippino_services \
     -v /home/michele/pippino_ws:/pippino_ws \
     --entrypoint=/pippino_ws/src/test_rtps_entrypoint.sh \
     --env CONTAINER=services \
     -p 7414:7414/udp -p 7415:7415/udp \
     pippino/ros2:services-$ROS_DISTRO
