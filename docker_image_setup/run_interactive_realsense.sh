#!/bin/bash

docker stop pippino_realsense
docker rm pippino_realsense

sudo /sbin/set_fan_pwm 90

if echo "$(uhubctl)" | grep -q "Port 2\: 0203 power 5gbps U0 enable connect \[8087\:0b37 Intel(R) Corporation Intel(R) RealSense(TM) Tracking Camera T265 224622112036\]"; then
    echo "T265 already correctly connected."
else
     sudo /sbin/remove_usb_device 1-2.3.2  # reset usb bus
     sleep 2
     uhubctl -l 1-2.3 -p 2 -a off  # T265
     sleep 2
     uhubctl -l 1-2.3 -p 2 -a on  # T265
fi


# ROS_DISTRO=humble

docker run -it --rm \
     -v /dev:/dev \
     --name pippino_realsense \
     --privileged \
     --runtime nvidia \
     --device-cgroup-rule "c 81:* rmw" \
     --device-cgroup-rule "c 189:* rmw" \
     -v /home/michele/pippino_ws:/pippino_ws \
     --net=host \
     --entrypoint="/bin/bash" \
     pippino/ros2:realsense-$ROS_DISTRO

# . /pippino_ws/install/local_setup.bash ;. /realsense_ws/install/local_setup.bash
# ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/esp32
# ros2 launch realsense2_camera rs_launch.py 
# ros2 launch pippino_bringup pippino.launch.py
# ros2 launch realsense2_camera rs_launch.py enable_gyro:=true
