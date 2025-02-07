# Autonomous

This repo is home to the Sooner Rover Team's [ROS 2](https://ros.org/) codebase. For more about this project and our plans, see [the Autonomous kanban board](https://github.com/orgs/Sooner-Rover-Team/projects/4/views/1).

## Join Us

Joining the Sooner Rover Team is simple! Just [hop in our Discord](https://discord.gg/qBHR26S6N5) and ask about the team you're interested in! It also doesn't hurt to [follow our Instagram](https://www.instagram.com/soonerrover/).

## Contributing

It can be difficult to know where to start. Luckily, we have a list of [projects you can work on](https://github.com/orgs/Sooner-Rover-Team/projects/4/views/1)! Some of these have hidden progression or difficulties, so please discuss with Autonomous leadership before you start working on something new.

For info about conventions (how we write/upload code, etc.), please see [our documentation on contributions](./CONTRIBUTING.md).

## Launch the Code

Launching the code is different depending on which Node or Launch File you're running. If you're just running a Rust Node individually, see the `Rust` section. Otherwise, continue with the instructions below...

### Python

To launch Python code, you typically use the `ros2` command. You'll need to source the ROS 2 scripts, which is done like so:

```bash
# you need to find where your ros2 `setup.bash` file is.
#
# this differs depending on how you installed it. 
# mine is here:
source /usr/lib64/ros2-jazzy/setup.bash

# next, grab the scripts' dependencies using uv, then hop in its venv
uv sync
. ./venv/bin/activate

# compile all ros2 packages using `colcon`
colcon build --symlink-install

# you'll get an `install/` folder inside the `auto_ros2` repo!
#
# now you can source the ROS 2 workspace scripts:
source install/local_setup.bash
```

Great, you're set up to run the code! Fair warning: **each time you open a new terminal, you'll need to `source` those files.**

```bash
# in ROS 2, you have to compile packages before your changes are made.
#
# that means using `colcon`, like so:
colcon build --symlink-install

# now, you're all good to run a node:
ros2 run node_name node_name
```

For info about installing any of those dependencies, including ROS 2, please see the [`CONTRIBUTING.md`](./CONTRIBUTING.md) file.
