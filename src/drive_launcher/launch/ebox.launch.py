"""
This file runs all required nodes to speak with the Ebox.
"""

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions.node import Node
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.node import get_logger


def generate_launch_description() -> LaunchDescription:
    # the `wheels_node` translates from `/cmd_vel` into messages we send to the
    # Ebox microcontrollers.
    #
    # it makes the Rover actually move... ;D
    wheels_node: Node = Node(
        name="wheels_node",
        package="wheels",
        executable="wheels_node",
    )

    # the `lights_node` provides a service to modify the LED strip's state.
    #
    # that's a required component of all rovers present at URC!
    lights_node: Node = Node(
        name="lights_node",
        package="lights",
        executable="lights_node",
    )

    # `sensors_node` quickly provides all data from sensors.
    #
    # it's the link between ROS 2 and the world
    sensors_node: Node = Node(
        name="sensors_node",
        package="sensors",
        executable="sensors_node",
    )

    # TODO: implement the camera launch files, then replace this!
    camera_warning: OpaqueFunction = OpaqueFunction(function=_warn_about_camera_todos)

    return LaunchDescription([wheels_node, lights_node, sensors_node, camera_warning])


def _warn_about_camera_todos(_):
    # we'll emit a diagnostic to remind ourselves lol
    logger: RcutilsLogger = get_logger("ebox.launch.py")
    _ = logger.error("TODO: implement cameras in launch file!")
