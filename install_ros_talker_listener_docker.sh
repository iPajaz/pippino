apt update && apt install -y curl gnupg2 lsb-release tcpdump nano
export CHOOSE_ROS_DISTRO=galactic
apt install -y ros-galactic-demo-nodes-cpp
export ROS_DOMAIN_ID=0
ros2 run demo_nodes_cpp listener

tcpdump -X -i any udp port 7400