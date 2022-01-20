#! /bin/bash


echo "Building images for librealsense version ${LIBRS_VERSION}"
sudo docker build \
        --build-arg LIBRS_VERSION=2.50.0 \
        -t pippino/ros2:rs50-v2 \
        -f Dockerfile.rs50 .
