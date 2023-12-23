#! /bin/bash

distro="galactic"

sudo docker build \
        --build-arg ROS_DISTRO=$distro \
        -t pippino/ros2:base-$distro \
        -f Dockerfile_ubu20_base .

