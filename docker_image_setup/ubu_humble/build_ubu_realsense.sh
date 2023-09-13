#! /bin/bash

ROS_DISTRO=humble
L4T_VERSION=35.3.1
BASE_IMAGE_NOROS=nvcr.io/nvidia/l4t-base:$L4T_VERSION
BASE_IMAGE=pippino/ros2:base-$ROS_DISTRO

sudo docker build \
        --build-arg LIBRS_VERSION=2.51.1 \
        --build-arg RS_WRAPPER_TAG=4.51.1 \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        --build-arg BASE_IMAGE_NOROS=$BASE_IMAGE_NOROS \
        -t pippino/ros2:realsense-$ROS_DISTRO \
        -f Dockerfile_ubu_realsense .
