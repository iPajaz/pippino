#!/bin/bash

docker exec -it pippino_services pkill ros2 -SIGINT

# Start scan
uhubctl -l 1-2.1.4 -p 1 -a on

docker stop pippino_services
docker rm pippino_services

docker run -it --rm \
     --name pippino_services \
     -v /dev:/dev \
     --runtime nvidia \
     --privileged \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     --entrypoint=/pippino_ws/src/pippino-services-entrypoint_interactive.sh \
     -v /dev/shm:/dev/shm \
     -e "RMW_IMPLEMENTATION" \
     pippino/ros2:services-$ROS_DISTRO
     # -v /home/michele/realsense_ws:/realsense_ws \
     # --net=host pippino/ros2:rs50-v2 bash

# . /pippino_ws/install/local_setup.bash ;. /realsense_ws/install/local_setup.bash
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
# ros2 launch realsense2_camera rs_launch.py 
# ros2 launch pippino_bringup pippino.launch.py
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true
