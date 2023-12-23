#!/bin/bash

cd ~/pippino_ws/src/docker_image_setup

docker exec -it pippino_services pkill ros2 -SIGINT

# docker stop pippino_services
# docker rm pippino_services

# docker stop pippino_realsense
# docker rm pippino_realsense
docker stop pippino_realsense_d455
docker rm pippino_realsense_d455
docker stop pippino_realsense_t265
docker rm pippino_realsense_t265

