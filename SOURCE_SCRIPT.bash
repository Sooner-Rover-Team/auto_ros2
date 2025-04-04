#!/usr/bin/env bash

echo -n "You should be running this script using `" && echo -n ". ./SOURCE_SCRIPT.fish" && echo "`."
echo "If you didn't do so, please restart the script."
echo

echo Sourcing virtual environment...
. .venv/bin/activate
echo Done!

echo Sourcing ROS 2 environment files...
source install/local_setup.bash
source /opt/ros/humble/setup.bash
echo Done!
