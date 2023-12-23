#! /bin/bash

ROS_DISTRO=galactic

BASE_IMAGE_NOROS=timongentzsch/l4t-ubuntu20-base:latest
BASE_IMAGE=pippino/ros2:base-$ROS_DISTRO

sudo docker build \
        --build-arg LIBRS_VERSION=2.53.1 \
        --build-arg RS_WRAPPER_TAG=4.51.1 \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        --build-arg BASE_IMAGE_NOROS=$BASE_IMAGE_NOROS \
        -t pippino/ros2:realsense-$ROS_DISTRO \
        -f Dockerfile_ubu20_realsense .
