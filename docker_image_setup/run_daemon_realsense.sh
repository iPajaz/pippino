#!/bin/bash

sudo docker stop pippino_realsense
sudo docker rm pippino_realsense

distro=galactic

sudo docker run -d \
     --name pippino_realsense \
     -v /dev:/dev \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     --privileged \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host pippino/ros2:realsense4 \
     bash -c ". /realsense_ws/install/local_setup.bash && ros2 launch realsense2_camera rs_launch.py config_file:=\'/pippino_ws/src/d455_wrapper4.0.yaml\'"

     # --restart always \


