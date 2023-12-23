#!/bin/bash

docker exec -it pippino_services pkill ros2 -SIGINT

docker stop pippino_services
docker rm pippino_services

docker run -d --rm \
     --name pippino_services \
     -v /dev:/dev \
     --runtime nvidia \
     --privileged \
     -v /home/michele/pippino_ws:/home/michele/pippino_ws \
     --net=host \
     -v /dev/shm:/dev/shm \
     -e "RMW_IMPLEMENTATION" \
     -e "FASTRTPS_DEFAULT_PROFILES_FILE" \
     -e "ROS_DISTRO" \
     -e "ROS_DOMAIN_ID" \
     pippino/ros2:services-$ROS_DISTRO
     # -e "RMW_IMPLEMENTATION" \
      # \
     # bash -c ". /opt/ros/$ROS_DISTRO/setup.bash && . /pippino_ws/install/local_setup.bash && . /services_ws/install/local_setup.bash && ros2 launch pippino_bringup pippino.launch.py"

     # --restart always \


