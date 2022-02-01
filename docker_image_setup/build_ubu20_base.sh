#! /bin/bash

sudo docker build \
        --build-arg ROS_DISTRO=foxy \
        -t pippino/ros2:base-foxy \
        -f Dockerfile_ubu20_base .

