from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # whether or not we'll launch using sim time.
    use_sim_time = LaunchConfiguration("use_sim_time")

    # launch non-`navigator` rover stuff
    rover: IncludeLaunchDescription = _rover_launch(use_sim_time)

    # start the manual control node
    manual_control_node: Node = _manual_control_node(use_sim_time)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            rover,
            manual_control_node,
        ]
    )


def _rover_launch(
    use_sim_time: LaunchConfiguration,
) -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    return IncludeLaunchDescription(
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
        ),
        launch_arguments=[("use_sim_time", use_sim_time)],
    )


def _manual_control_node(use_sim_time: LaunchConfiguration) -> Node:
    return Node(
        executable="manual_control_gui_node",
        package="manual_control",
        name="manual_control_gui_node",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
