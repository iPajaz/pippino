#! /bin/bash

sudo docker build \
        --build-arg LIBRS_VERSION=2.50.0 \
        --build-arg RS_WRAPPER_TAG=3.2.3 \
        --build-arg BASE_IMAGE=dustynv/ros:galactic-ros-base-l4t-r32.6.1 \
        -t pippino/ros2:realsense-galactic \
        -f Dockerfile_ubu18_realsense .
