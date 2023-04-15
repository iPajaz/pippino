#! /bin/bash

distro="galactic"

sudo docker build \
        --build-arg ROS_DISTRO=$distro \
        --build-arg BASE_IMAGE=pippino/ros2:base-$distro \
        -t pippino/ros2:services-$distro \
        -f Dockerfile_ubu20_services .

