#!/bin/bash

source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

echo "Starting AGV Bringup..."

gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch agv_bringup robot.launch.py
exec bash
"

sleep 5

echo "Starting MQTT Bridge..."

gnome-terminal -- bash -c "
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch agv_localization agv_bringup_with_mqtt_bridge.launch.py
exec bash
"
