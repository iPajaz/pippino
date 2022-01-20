#! /bin/bash

sudo docker build \
        -t pippino/ros2:full-ubu18 \
        -f Dockerfile.full .
