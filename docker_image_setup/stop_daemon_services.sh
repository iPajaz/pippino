#!/bin/bash

docker exec -it pippino_services pkill ros2 -SIGINT

docker stop pippino_services
docker rm pippino_services

