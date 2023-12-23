#!/bin/bash

docker stop pippino_realsense_t265
docker rm pippino_realsense_t265

if echo "$(uhubctl)" | grep -q "Port 1\: 0203 power 5gbps U0 enable connect \[8087\:0b37 Intel(R) Corporation Intel(R) RealSense(TM) Tracking Camera T265 224622112036\]"; then
    echo "T265 already correctly connected."

else
     sudo /sbin/remove_usb_device 1-2.4.1  # reset usb bus
     sleep 2
     uhubctl -l 1-2.4 -p 1 -a off  # T265
     sleep 2
     uhubctl -l 1-2.4 -p 1 -a on  # T265
fi

docker run -d --rm \
     -v /dev:/dev \
     --name pippino_realsense_t265 \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/home/michele/pippino_ws \
     --net=host \
     -v /dev/shm:/dev/shm \
     -e "ROS_DISTRO" \
     -e "ROS_DOMAIN_ID" \
     -e "ROS_DISCOVERY_SERVER" \
     --entrypoint=/home/michele/pippino_ws/src/pippino-realsense-t265-entrypoint.sh \
     pippino/ros2:realsense-rsusb-$ROS_DISTRO
     # -e "RMW_IMPLEMENTATION" \
     # --restart always \


