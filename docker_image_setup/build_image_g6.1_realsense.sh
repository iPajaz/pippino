#! /bin/bash

sudo docker build \
        --build-arg LIBRS_VERSION=2.50.0 \
        --build-arg RS_WRAPPER_TAG=3.2.3 \
        -t pippino/ros2:g6.1-realsense \
        -f Dockerfile_realsense .
