#!/bin/bash

export FASTRTPS_DEFAULT_PROFILES_FILE=/home/michele/pippino_ws/src/fastrtps_client.xml 
ros2 daemon stop; ros2 daemon start

ros2 run rosbridge_server rosbridge_websocket