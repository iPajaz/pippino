#!/bin/bash

cd ~/pippino_ws/src/docker_image_setup
# echo 90 > sudo tee /sys/devices/pwm-fan/target_pwm
sudo /sbin/set_fan_pwm 90

# sleep 2

# uhubctl -l 2-1.3 -p 1 -a cycle  # D455
# uhubctl -l 2-1.3.4 -p 2 -a cycle  # rplidar

if echo "$(uhubctl)" | grep -q "Port 2\: 0203 power 5gbps U0 enable connect \[8087\:0b37 Intel(R) Corporation Intel(R) RealSense(TM) Tracking Camera T265 224622112036\]"; then
    echo "T265 already correctly connected."

	# sudo /sbin/remove_usb_device 2-1.3.2  # reset usb bus
	# sleep 2
	# uhubctl -l 2-1.3 -p 2 -a off  # T265
	# sleep 2
	# uhubctl -l 2-1.3 -p 2 -a on  # T265
else
	sudo /sbin/remove_usb_device 1-2.3.2  # reset usb bus
	sleep 2
	uhubctl -l 1-2.3 -p 2 -a off  # T265
	sleep 2
	uhubctl -l 1-2.3 -p 2 -a on  # T265
fi

sleep 2
./run_daemon_services.sh
sleep 4
./run_daemon_realsense.sh


# To turn off and reset D455
	# sudo /sbin/remove_usb_device 2-1.3.1  # reset usb bus
	# sleep 2
	# uhubctl -l 2-1.3 -p 1 -a off  # D455
	# sleep 2
	# uhubctl -l 2-1.3 -p 1 -a on  # D455


# ports:
# 1-2.3.4 p2: lidar
# 2-1.3 p1: D455
# 2-1.3 p2: T265