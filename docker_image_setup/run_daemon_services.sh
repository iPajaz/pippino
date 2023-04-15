#!/bin/bash

docker exec -it pippino_services pkill ros2 -SIGINT

docker stop pippino_services
docker rm pippino_services

distro=humble

docker run -d \
     --name pippino_services \
     -v /dev:/dev \
     --runtime nvidia \
     --privileged \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     pippino/ros2:services-$distro
      # \
     # bash -c ". /opt/ros/$distro/setup.bash && . /pippino_ws/install/local_setup.bash && . /services_ws/install/local_setup.bash && ros2 launch pippino_bringup pippino.launch.py"

     # --restart always \


