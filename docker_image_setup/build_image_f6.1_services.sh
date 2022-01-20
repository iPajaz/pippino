#! /bin/bash

sudo docker build \
        -t pippino/ros2:f6.1-services \
        -f Dockerfile_services .
