# Pippino II main ROS repo

This repo includes all the launch files used to operate the Pippino II cleaning autonomous mobile robot.

The `eightball-code` branch includes all the code running on the control PC, as currently the Jetson Nano installed on the robot cannot handle the whole navigation stack and related packages.

The `jetson-code` branch includes all the required code running on the Jetson Nano.

Packages related to autodocking, environment discovery and initial self-localization are contained in separate repositories.
