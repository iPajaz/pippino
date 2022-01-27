#! /bin/bash

sudo docker build \
        --build-arg LIBRS_VERSION=2.50.0 \
        --build-arg RS_WRAPPER_TAG=3.2.3 \
        -t pippino/ros2:full \
        -f Dockerfile_ubu20_full .

