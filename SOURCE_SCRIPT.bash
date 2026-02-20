#!/usr/bin/env bash
set -e

GREEN="\e[32m"
RESET="\e[0m"

echo -n "You should be running this script using \`" && echo -n "$GREEN. ./SOURCE_SCRIPT.bash$RESET" && echo "\`."
echo "If you didn't do so, please restart the script."
echo

echo "Sourcing ROS 2 environment files..."
. install/local_setup.bash || true
echo "Done!"
echo

echo "Setting \`ROS_DISTRO\` environment variable as required by \`safe_drive\`..."
export ROS_DISTRO=humble
echo "Done!"
echo

echo "Setting Nav2-compatible ROS middleware implementation..."
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "Done!"
echo

echo "Turning on colorized output from ROS 2..."
export RCUTILS_COLORIZED_OUTPUT=1
echo "Done!"
echo

echo "All source tasks are now complete!"
