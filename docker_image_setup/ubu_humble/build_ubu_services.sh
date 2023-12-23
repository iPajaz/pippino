#! /bin/bash

ROS_DISTRO=humble
BASE_IMAGE=pippino/ros2:base-$ROS_DISTRO

sudo docker build \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        -t pippino/ros2:services-$ROS_DISTRO \
        -f Dockerfile_ubu_services .

