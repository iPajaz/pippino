#!/bin/bash

cd ~/pippino_ws/src/docker_image_setup

sudo docker exec -it pippino_services pkill ros2 -SIGINT

sudo docker stop pippino_services
sudo docker rm pippino_services

sudo docker stop pippino_realsense
sudo docker rm pippino_realsense

