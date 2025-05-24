"""
Starts everything the Rover needs to Autonomously navigate.

(except the `navigator_node`. that's on you...)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions.find_package import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    return LaunchDescription(
        [
            # `rover.launch.py` is the ros 2 side of things.
            #
            # it includes stuff like Nav2 and `slam_toolbox` for interpreting
            # and responding to our sensory information
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                pkg_drive_launcher,
                                "launch",
                                "rover.launch.py",
                            ]
                        )
                    ]
                )
            ),
            #
            # and `ebox.launch.py` is the ebox <=> autonomous communication
            # stuff (primarily rust packages)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                pkg_drive_launcher,
                                "launch",
                                "ebox.launch.py",
                            ]
                        )
                    ]
                )
            ),
        ]
    )
