# Simulator

This simulator uses Gazebo Fortress (for ROS 2 Humble).

## Usage

`colcon build && ros2 launch simulator sim.launch.py`.

You can debug sensors and such using either ROS 2 or Gazebo. They require a translation layer known as the [`ros_gz_bridge`](https://github.com/gazebosim/ros_gz) to transform ROS 2 messages into Gazebo messages. To echo a Gazebo topic (which is NOT a ROS 2 topic), you can use: `ign topic -t /imu -e`

If you'd like to send wheel speeds or whatever to the simulator manually, you can use ROS 2: `ros2 topic pub /control/wheels custom_interfaces/msg/WheelsMessage "{left_wheels: 26.0, right_wheels: 26.0}" --once`
