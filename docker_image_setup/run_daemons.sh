#!/bin/bash

cd ~/pippino_ws/src/docker_image_setup
# echo 90 > sudo tee /sys/devices/pwm-fan/target_pwm
# sudo /sbin/set_fan_pwm 90

# Start scan
uhubctl -l 1-2.4.4 -p 1 -a on

# sleep 2

# uhubctl -l 2-1.2 -p 1 -a cycle  # D455
# uhubctl -l 2-1.2.4 -p 2 -a cycle  # rplidar

if echo "$(uhubctl)" | grep -q "Port 1\: 0203 power 5gbps U0 enable connect \[8087\:0b37 Intel(R) Corporation Intel(R) RealSense(TM) Tracking Camera T265 224622112036\]"; then
    echo "T265 already correctly connected."

else
	sudo /sbin/remove_usb_device 1-2.4.1  # reset usb bus
	sleep 2
	uhubctl -l 1-2.4 -p 1 -a off  # T265
	sleep 2
	uhubctl -l 1-2.4 -p 1 -a on  # T265
fi

sleep 2
./run_daemon_services.sh
sleep 2
# ./run_daemon_realsense.sh
./run_daemon_realsense_d455.sh
sleep 2
./run_daemon_realsense_t265.sh

