#!/bin/bash

     # --name pippino_realsense \
sudo docker run -it --rm \
     -v /dev:/dev \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host pippino/ros2:realsense bash
     # -v /home/michele/realsense_ws:/realsense_ws \
     # --net=host pippino/ros2:rs50-v2 bash

# . /pippino_ws/install/local_setup.bash ;. /realsense_ws/install/local_setup.bash
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
# ros2 launch realsense2_camera rs_launch.py 
# ros2 launch pippino_bringup pippino.launch.py
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true
