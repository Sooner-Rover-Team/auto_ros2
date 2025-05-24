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

    # set gazebo plugin search path
    #
    # note: name is IGN_GAZEBO_SYSTEM_PLUGIN_PATH for our old-ass Gazebo
    # Fortress (6), but for newer versions, you'll want to use
    # `GAZEBO_PLUGIN_PATH` instead.
    #
    # anyway. let's gonna grab the ros installation path and add it onto
    # there...
    ROS_INSTALLATION_PATH="$(cat {{ros2_workspace_dir}}/.ros_path)$(cat {{ros2_workspace_dir}}/.ros_distro)/lib"
    export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$ROS_INSTALLATION_PATH
    echo "Adding these paths to Gazebo's plugin path: $IGN_GAZEBO_SYSTEM_PLUGIN_PATH"

    # we'll also kill the simulator if it's still alive (happens pretty often
    # from previous runs)
    (killall -9 gazebo & killall -9 gzserver & killall -9 gzclient & killall -9 ign & killall -9 ruby & killall -9 simulator) || true

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
    #!/usr/bin/env bash
    set -e
    echo "Fetching newest dependencies..."

    # grab the ros distro we're using
    export ROS_DISTRO=$(cat {{ros2_workspace_dir}}/.ros_distro)
    export ROS_PYTHON_VERSION=3

    # check which os we are
    OS=$(cat /etc/os-release | grep '^ID=' | sed 's/^ID=//' | awk '{print tolower($0)}')

    # check whether we want to use `sudo` to update/install
    OPTIONAL_SUDO=""
    if [[ $(id -u) -ne 0 ]]; then
        OPTIONAL_SUDO="sudo"
        echo "If prompted, please type in your password to update the system package cache..."
    fi

    # grab the package manager we'll use
    PKG_MANAGER_FETCH="$OPTIONAL_SUDO apt-get update -y"
    PKG_MANAGER_INSTALL="$OPTIONAL_SUDO apt-get install -y --no-install-recommends"
    PKG_MANAGER_INSTALL_FLAGS=""
    if [[ "$OS" == *"fedora"* ]]; then
        PKG_MANAGER_FETCH="dnf check-update || true"
        PKG_MANAGER_INSTALL="$OPTIONAL_SUDO dnf install -y"
        PKG_MANAGER_INSTALL_FLAGS="--skip-unavailable"
    fi

    echo "Fetching system packages..."
    $PKG_MANAGER_FETCH
    echo "System package repo cache updated!"

    echo "Updating the virtual environment..."
    uv sync
    echo "The virtual environment is now up-to-date."

    # note: we shim an ubuntu version to match the given ROS 2 distro (ver)
    UBUNTU_SHIM_VERSION="ubuntu:jammy"
    if [[ "$ROS_DISTRO" == "humble" ]]; then
        UBUNTU_SHIM_VERSION="ubuntu:jammy"
    elif [[ "$ROS_DISTRO" == "jazzy" ]]; then
        UBUNTU_SHIM_VERSION="ubuntu:noble"
    elif [[ "$ROS_DISTRO" == "kilted" ]]; then
        UBUNTU_SHIM_VERSION="ubuntu:noble"
    fi

    # we're going to grab a list of packages in a kinda-hacky way, but it
    # allows us to more easily support Fedora (and, potentially, other systems)
    echo "Looking for packages on ROS 2 ($ROS_DISTRO)"
    ROS2_PKG_PREFIX="ros-$ROS_DISTRO-"
    rosdep update --rosdistro $ROS_DISTRO
    RAW_PKGS=$(rosdep keys --from-paths {{ros2_workspace_dir}}/src | xargs)
    PACKAGE_LIST=$(rosdep resolve --os=$UBUNTU_SHIM_VERSION $RAW_PKGS | sed -E 's/#[^ ]+//g' | xargs)
    echo "Found the following packages: $PACKAGE_LIST"
    echo

    echo "Installing packages via system package manager..."
    $PKG_MANAGER_INSTALL $PACKAGE_LIST $PKG_MANAGER_INSTALL_FLAGS
    echo "Completed installing system dependencies!"

    echo "Sourcing the environment..."
    . {{ros2_workspace_dir}}/SOURCE_SCRIPT.bash
    echo "Environment sourced!"

    echo
    echo "Dependencies have been installed. You may now use ROS 2."
