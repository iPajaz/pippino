#!/bin/bash

cd ~/pippino_ws/src/docker_image_setup

sleep 2

uhubctl -l 1-2 -p 1 -a cycle

sleep 4

./run_daemon_services.sh
./run_daemon_realsense.sh
