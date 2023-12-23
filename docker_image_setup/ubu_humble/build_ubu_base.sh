#! /bin/bash

ROS_DISTRO=humble

BASE_IMAGE=nvcr.io/nvidia/l4t-jetpack:r35.4.1

sudo docker build \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        -t pippino/ros2:base-$ROS_DISTRO \
        -f Dockerfile_ubu_base .

