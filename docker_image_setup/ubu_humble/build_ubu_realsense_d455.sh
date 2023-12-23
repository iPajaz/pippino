#! /bin/bash

ROS_DISTRO=humble
BASE_IMAGE_NOROS=nvcr.io/nvidia/l4t-jetpack:r35.4.1
BASE_IMAGE=pippino/ros2:base-$ROS_DISTRO

sudo docker build \
        --build-arg LIBRS_VERSION=2.54.1 \
        --build-arg RS_WRAPPER_TAG=4.54.1 \
        --build-arg ROS_DISTRO=$ROS_DISTRO \
        --build-arg CAM_TYPE=d455 \
        --build-arg BASE_IMAGE=$BASE_IMAGE \
        --build-arg BASE_IMAGE_NOROS=$BASE_IMAGE \
        --build-arg ENTRYPOINT_SCRIPT="pippino-realsense-d455-entrypoint.sh" \
        --build-arg UID=$(id -u) \
        --build-arg GID=$(id -g) \
        -t pippino/ros2:realsense-rsusb-d455-$ROS_DISTRO \
        -f Dockerfile_ubu_realsense .
