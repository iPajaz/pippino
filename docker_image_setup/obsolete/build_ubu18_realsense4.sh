#! /bin/bash

sudo docker build \
        --build-arg LIBRS_VERSION=2.50.0 \
        --build-arg RS_WRAPPER_TAG=ros2-beta \
        --build-arg ROS_DISTRO=foxy \
        --build-arg BASE_IMAGE=dustynv/ros:foxy-ros-base-l4t-r32.6.1 \
        -t pippino/ros2:realsense4 \
        -f Dockerfile_ubu18_realsense .
