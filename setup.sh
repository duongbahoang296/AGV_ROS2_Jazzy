#!/bin/bash

set -e

source /opt/ros/jazzy/setup.bash

cd ros2_ws

echo "Building workspace..."

colcon build --symlink-install

echo "Sourcing workspace..."

source install/setup.bash

echo ""
echo "Workspace Ready!"
