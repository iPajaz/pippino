#!/bin/bash

sudo docker stop pippino_services
sudo docker rm pippino_services

sudo docker run -it \
     --name pippino_services \
     -v /dev:/dev \
     --runtime nvidia \
     --privileged \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host pippino/ros2:services \
     bash -c ". /opt/ros/galactic/setup.bash && . /pippino_ws/install/local_setup.bash && . /services_ws/install/local_setup.bash && ros2 launch pippino_bringup pippino.launch.py"

     # --restart always \


