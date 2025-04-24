# use fish for scripting
set shell := ["bash", "-c"]

ros2_workspace_dir := justfile_directory()

# aliases let users provide multiple commands for one
alias fetch := get
alias update := get

# Prints this help message.
help:
    @ echo "Welcome to the SoRo Justfile! Here are the commands you can run:"
    @ just --list

# Builds the project using ROS 2 tooling.
build:
    @ echo "Building the ROS 2 workspace...";
    colcon build --symlink-install
    @ echo "Build complete!";

# Runs the Gazebo simulation.
sim: build
    #!/usr/bin/env bash

    # so... we've just completed the build.
    #
    # let's source any new packages it made
    . ./SOURCE_SCRIPT.bash

    # we'll also kill the simulator if it's still alive (happens pretty often
    # from previous runs)
    killall -9 gazebo & killall -9 gzserver & killall -9 gzclient & killall -9 ign & killall -9 ruby & killall -9 simulator

    # start the simulator by running its launch file!
    echo "The simulation is about to begin...";
    ros2 launch simulator sim.launch.py

# Removes extra build files.
clean:
    @ echo "Attempting to clean build files...";
    rm -rfd install/ install/ log/ .cargo/
    rm -rfd build/
    - rm -rfd .venv/
    @ echo "Build files have been removed!";

# Runs our test suite through `colcon`.
test: build
    @ echo "Running tests..."
    colcon test --event-handlers console_direct+
    @ echo "Test run complete!"

# Fetches all the newest dependencies. (including `rosdep`)
get:
    @ echo "Fetching newest dependencies..."
    @ echo "If prompted, please type in your password..."
    sudo apt update
    uv sync
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
    @ echo "Dependencies fetched successfully!"
