#!/bin/bash

sudo docker stop pippino_services
sudo docker rm pippino_services

sudo docker run -it --rm \
     --name pippino_services \
     -v /dev:/dev \
     --runtime nvidia \
     --privileged \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host pippino/ros2:services bash
     # -v /home/michele/realsense_ws:/realsense_ws \
     # --net=host pippino/ros2:rs50-v2 bash

# . /pippino_ws/install/local_setup.bash ;. /realsense_ws/install/local_setup.bash
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
# ros2 launch realsense2_camera rs_launch.py 
# ros2 launch pippino_bringup pippino.launch.py
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true
