#!/bin/bash

cd ~/pippino_ws/src/docker_image_setup

docker exec -it pippino_services pkill ros2 -SIGINT

docker stop pippino_services
docker rm pippino_services

docker stop pippino_realsense
docker rm pippino_realsense
