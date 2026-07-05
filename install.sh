#!/bin/bash

set -e

echo "========================================"
echo "Updating package list..."
echo "========================================"

sudo apt update

echo "========================================"
echo "Installing required Ubuntu packages..."
echo "========================================"

sudo apt install -y \
git \
curl \
python3-pip \
python3-opencv \
python3-yaml \
python3-colcon-common-extensions \
python3-vcstool

echo "========================================"
echo "Installing ROS2 packages..."
echo "========================================"

sudo apt install -y \
ros-jazzy-cv-bridge \
ros-jazzy-tf2-ros \
ros-jazzy-usb-cam \
ros-jazzy-micro-ros-agent

echo "========================================"
echo "Installing Python packages..."
echo "========================================"

python3 -m pip install --upgrade pip

python3 -m pip install \
numpy \
paho-mqtt

echo "========================================"
echo "Installation completed."
echo "========================================"
echo "Installing ROS dependencies..."

rosdep install \
--from-paths ros2_ws/src \
--ignore-src \
-r \
-y
