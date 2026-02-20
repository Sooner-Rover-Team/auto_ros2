#!/usr/bin/env fish

set ROS2_WORKSPACE_LOCATION (dirname (readlink -m (status --current-filename)))

echo -n "You should be running this script using `" && set_color green && echo -n ". ./SOURCE_SCRIPT.fish" && set_color normal && echo "`."
echo "If you didn't do so, please restart the script."
echo

echo "Sourcing ROS 2 environment files..."
bass source $ROS2_WORKSPACE_LOCATION/install/local_setup.bash || true
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
