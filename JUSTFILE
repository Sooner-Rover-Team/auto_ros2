# use fish for scripting
set shell := ["bash", "-c"]

ros2_workspace_dir := justfile_directory()

build:
    @ echo "Building the ROS 2 workspace...";
    colcon build --symlink-install

sim: build
    - . ./SOURCE_SCRIPT.bash & killall -9 gazebo & killall -9 gzserver & killall -9 gzclient & killall -9 ign & killall -9 ruby
    @ echo "The simulation is about to begin...";
    ros2 launch simulator sim.launch.py

clean:
    deactivate
    rm -rfd install/ install/ log/ .venv/ .cargo/
