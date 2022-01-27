#! /bin/bash

sudo docker build \
        -t pippino/ros2:services \
        -f Dockerfile_ubu20_services .

