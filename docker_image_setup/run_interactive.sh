#!/bin/bash

sudo docker run -it --rm \
     --name crazy_pippino \
     -v /dev:/dev \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     --device=/dev/esp32 \
     -v /home/michele/pippino_ws:/pippino_ws \
     -v /home/michele/realsense_ws:/realsense_ws \
     --net=host pippino/ros2:full-ubu18 bash
     # --device="/dev/iio\:device0" --device="/dev/iio\:device1" --device="/dev/iio\:device2" \

# . /pippino_ws/install/local_setup.bash ;. /realsense_ws/install/local_setup.bash
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
# ros2 launch realsense2_camera rs_launch.py 
# ros2 launch pippino_bringup pippino.launch.py
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true
