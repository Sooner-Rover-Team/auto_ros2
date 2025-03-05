# use fish for scripting
set shell := ["fish", "-c"]

ros2_workspace_dir := justfile_directory()

build:
    @ set_color brgreen black; echo "Building the ROS 2 workspace..."; set_color normal
    colcon build --symlink-install

sim: build
    - . ./SOURCE_SCRIPT.fish & killall -9 gazebo & killall -9 gzserver & killall -9 gzclient & killall -9 ign & killall -9 ruby
    @ set_color brgreen black; echo "The simulation is about to begin..."; set_color normal
    ros2 launch simulator sim.launch.py

clean:
    deactivate
    rm -rfd install/ install/ log/ .venv/ .cargo/
