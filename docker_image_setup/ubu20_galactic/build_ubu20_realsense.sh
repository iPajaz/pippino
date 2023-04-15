#! /bin/bash

ROS_DISTRO=galactic
L4T_VERSION=32.7.1
# BASE_IMAGE=dustynv/ros:$ROS_DISTRO-ros-base-l4t-r$L4T_VERSION
BASE_IMAGE_NOROS=timongentzsch/l4t-ubuntu20-base:latest
BASE_IMAGE=pippino/ros2:base-$ROS_DISTRO

sudo docker build \
        --build-arg LIBRS_VERSION=2.53.1 \
        --build-arg RS_WRAPPER_TAG=4.51.1 \
        --build-arg L4T_VERSION=$L4T_VERSION \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        --build-arg BASE_IMAGE_NOROS=$BASE_IMAGE_NOROS \
        -t pippino/ros2:realsense-$ROS_DISTRO \
        -f Dockerfile_ubu18_realsense .
