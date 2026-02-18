"""
This file runs all required nodes to speak with the Ebox.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions.node import Node


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

    # Unitree L2 LiDAR bridge package.
    pkg_soro_lidar: str = get_package_share_directory("soro_lidar")
    lidar_launch: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_soro_lidar,
                        "launch.py",
                    ]
                )
            ]
        )
    )

    # See3CAM color stream for ArUco marker detection.
    pkg_see3cam: str = get_package_share_directory("see3cam")
    see3cam_launch: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_see3cam,
                        "launch",
                        "see3cam.launch.py",
                    ]
                )
            ]
        )
    )

    return LaunchDescription(
        [wheels_node, lights_node, sensors_node, lidar_launch, see3cam_launch]
    )
