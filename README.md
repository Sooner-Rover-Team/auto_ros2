# Autonomous

This repo is home to the Sooner Rover Team's [ROS 2](https://ros.org/) codebase. For more about this project and our plans, see [the Autonomous kanban board](https://github.com/orgs/Sooner-Rover-Team/projects/4/views/1).

## Join Us

Joining the Sooner Rover Team is simple! Just [hop in our Discord](https://discord.gg/qBHR26S6N5) and ask about the team you're interested in! It also doesn't hurt to [follow our Instagram](https://www.instagram.com/soonerrover/).

## Contributing

It can be difficult to know where to start. Luckily, we have a list of [projects you can work on](https://github.com/orgs/Sooner-Rover-Team/projects/4/views/1)! Some of these have hidden progression or difficulties, so please discuss with Autonomous leadership before you start working on something new.

For info about conventions (how we write/upload code, etc.), please see [our documentation on contributions](./CONTRIBUTING.md).

## Launch the Code

Unless you want to suffer, you'll need the SoRo Docker container set up on your computer before continuing. Please [see its documentation](./docker/README.md) to set it up, then come back when you're done.

Okay, all done? Let's run the code!

First, ensure you're looking at the desktop of the Docker container. It should be red, and you'll see the SoRo logo at the bottom left of the screen.

Click on that SoRo icon, then press 'Terminal Emulator'. When you're in its terminal, run each of these commands:

```bash
# for reference, these gray hashtag things are comments - you don't need to run
# them!
#
# anyway, we're starting off in the `/home/soro` folder.
#
# we'll want to navigate to the code you downloaded, which is shared between
# the container and your "host" computer.
#
# to do that, use `cd` (for Change Directory):
soro@71b7d4dde026 ~> cd auto_ros2
soro@71b7d4dde026 ~/auto_ros2 (docs/april_pass)>

# alright, we're in the code folder! let's list the files...
#
# it should look something like this:
soro@71b7d4dde026 ~/auto_ros2 (docs/april_pass)> ls
bin    cam.yml     Cargo.toml       docker  install   launch  msg         pyproject.toml  SOURCE_SCRIPT.bash  src
build  Cargo.lock  CONTRIBUTING.md  docs    JUSTFILE  log     output.out  README.md       SOURCE_SCRIPT.fish  uv.lock

# ok, let's start by grabbing the code's dependencies.
#
# to do so, we use a tool called `uv`...
soro@71b7d4dde026 ~/auto_ros2 (docs/april_pass)> uv sync
Resolved 90 packages in 9ms
Audited 68 packages in 0.02ms

# we have the dependencies now!
#
# we'll use the "fish source script" to grab all the ROS 2 stuff.
#
# you may get an error saying that there's "no such file or directory", but
# that's completely fine.
#
# (it just means you haven't built the code yet)
soro@71b7d4dde026 ~/auto_ros2 (docs/april_pass)> . ./SOURCE_SCRIPT.fish
"You should be running this script using `. ./SOURCE_SCRIPT.fish`.
If you didn't do so, please restart the script.

Sourcing virtual environment...
Done!
Sourcing ROS 2 environment files...
bass: line 1: install/local_setup.bash: No such file or directory
Done!"
(autonomous) soro@71b7d4dde026 ~/auto_ros2 (docs/april_pass)>

# speaking of building the code... let's do that!
(autonomous) soro@71b7d4dde026 ~/auto_ros2 (docs/april_pass)> just build
Building the ROS 2 workspace...
colcon build --symlink-install
Starting >>> aruco_node
Starting >>> custom_interfaces
Starting >>> lights_node
Starting >>> log_node
Starting >>> navigator_node
Starting >>> sensors_node
Starting >>> simulator
Starting >>> wheels_node
Finished <<< custom_interfaces [1.36s]
Finished <<< log_node [1.66s]
Finished <<< aruco_node [1.73s]
Finished <<< navigator_node [1.76s]
Finished <<< simulator [1.77s]
Finished <<< lights_node [4.74s]
Finished <<< wheels_node [4.84s]
Finished <<< sensors_node [4.86s]

Summary: 8 packages finished [5.05s]

# the code built successfully! we can now run the simulator:
(autonomous) soro@71b7d4dde026 ~/auto_ros2 (docs/april_pass)> just sim
Building the ROS 2 workspace...
colcon build --symlink-install
# (snip...)

Summary: 8 packages finished [3.70s]

#(snip...)
The simulation is about to begin...
ros2 launch simulator sim.launch.py
Done!

# you should now see the Gazebo simulator on your screen!
#
# watch the Rover complete its goals, and feel free to change the code to make
# it do something else! :)
```

You should see the Rover moving around in a little simulation! Outside the container, on your real system, you can make changes to the code. Re-run the simulator using the same `just sim` command, and you'll see your changes applied!

If you experience any issues, please see our [Frequently Asked Questions page](./docs/faq.md)! Or, you can [reach out on Discord in `#autonomous`](https://discord.gg/AD6ZR8nkuq). :D

## Local Installation and Best Practices

For info about installing dependencies on your local system, including ROS 2, please see the [`CONTRIBUTING.md`](./CONTRIBUTING.md) file.
