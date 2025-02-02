#!/usr/bin/env fish

echo -n "You should be running this script using `" && set_color green && echo -n ". ./SOURCE_SCRIPT.fish" && set_color normal && echo "`."
echo "If you didn't do so, please restart the script."
echo

echo Sourcing virtual environment...
. src/log_node/.venv/bin/activate.fish
echo Done!

echo Sourcing ROS 2 environment files...
bass source install/local_setup.bash
bass source /opt/ros/humble/setup.bash
echo Done!
