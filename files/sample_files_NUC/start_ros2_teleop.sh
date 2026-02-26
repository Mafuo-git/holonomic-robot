#!/bin/bash

echo "Attente du réseau..."

# Attendre qu'une interface non-loopback ait une IP
while ! ip -4 addr show scope global | grep -q inet; do
    sleep 1
done

echo "Adresse IP détectée"

# Optionnel : attendre une route par défaut
while ! ip route | grep -q default; do
    sleep 1
done

echo "Route par défaut détectée"

echo "Réseau prêt"

# petit délai de sécurité
sleep 2


# Sécurité
set -e

# load the ROS 2 environment
source /opt/ros/jazzy/setup.bash
source ~/holonomic-robot/files/ros2_ws/install/setup.bash

# environnement ROS variables (allows communication between nodes to send and receive messages from the raspberry and other devices)
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# (optional) log
echo "ROS2 démarré à $(date)" >> ~/ros2_startup.log

# the launchfile (teleop with node red)
exec ros2 launch cpp_motors_control teleop_node_red.launch.py
