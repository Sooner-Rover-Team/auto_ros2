# `manual_control`

This ROS 2 package includes a node to "manually control" the Rover using the Autonomous navigation stack. (useful, huh?)

It's only for testing - please don't use this for anything important.

## Setup

Nothing special needed here. Just follow the typical setup steps in our [README](../../README.md). Ensure ROS 2 is installed on your computer, or that it's using the Docker container. **This is intended to be ran on your computer - not the Rover!**

## Nodes

At the moment, we've just got one node...

1. `manual_control::manual_control_gui_node`: A [`pygame`](https://www.pygame.org/docs/) interface to control the Rover. Quick and dirty, primarily for testing Autonomous more easily.

- Built in `pygame` for simplicity's sake :)

## Usage

If you've already launched Autonomous control, you can also start the `manual_control_gui_node` with `ros2 run manual_control manual_control_gui_node`. **This is only for usage in the simulator** because this node is made to be used on a computer that _isn't_ the Jetson.

Otherwise, you can start all the non-Autonomous stuff alongside manual control with `ros2 launch manual_control manual_control.launch.py`. Please ensure Autonomous isn't already running - otherwise, you'll get multiple nodes running for the same purpose!

- **Left joystick**: turn left and right
- **LT** and **RT**: go backward and forward, respectively
