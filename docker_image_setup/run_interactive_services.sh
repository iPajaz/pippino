#!/bin/bash

docker exec -it pippino_services pkill ros2 -SIGINT

docker stop pippino_services
docker rm pippino_services

distro=galactic

docker run -it --rm \
     --name pippino_services \
     -v /dev:/dev \
     --runtime nvidia \
     --privileged \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     --entrypoint=/pippino_ws/src/pippino-services-entrypoint_interactive.sh \
     pippino/ros2:services-$distro
     # -v /home/michele/realsense_ws:/realsense_ws \
     # --net=host pippino/ros2:rs50-v2 bash

# . /pippino_ws/install/local_setup.bash ;. /realsense_ws/install/local_setup.bash
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
# ros2 launch realsense2_camera rs_launch.py 
# ros2 launch pippino_bringup pippino.launch.py
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true
