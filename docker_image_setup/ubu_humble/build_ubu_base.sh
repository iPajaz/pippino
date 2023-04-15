#! /bin/bash

ROS_DISTRO=humble
L4T_VERSION=35.3.1
BASE_IMAGE=nvcr.io/nvidia/l4t-base:$L4T_VERSION

sudo docker build \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        -t pippino/ros2:base-$ROS_DISTRO \
        -f Dockerfile_ubu_base .

