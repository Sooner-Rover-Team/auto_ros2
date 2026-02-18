from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from navigator_node.types import (
    NavigationMode,
)


def generate_launch_description():
    pkg_simulator: str = get_package_share_directory("simulator")
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    use_sim_time: str = "True"

    # start the rover launch script
    rover_launch_file: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "rover.launch.py",
                    ]
                )
            ],
        ),
        launch_arguments=[
            ("use_sim_time", use_sim_time),
            ("ros2_control_plugin", "gz_ros2_control/GazeboSimSystem"),
            ("use_gazebo_ros2_control", "true"),
            ("start_controller_manager", "false"),
        ],
    )

    # launch all the gazebo stuff
    gazebo_launch_file: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_simulator,
                        "launch",
                        "gazebo_only.launch.py",
                    ]
                )
            ],
        ),
        launch_arguments=[("use_sim_time", use_sim_time)],
    )

    # note: EXTREMELY IMPORTANT THAT THESE MATCH THE SIMULATOR'S STARTING
    # COORDINATE!!!
    START_LAT: float = 38.407258
    START_LON: float = -110.795204

    # add some coordinate to go to...
    latitude: float = START_LAT - 0.002  # 0.0002
    longitude: float = START_LON - 0.002  # 0.0003

    mode_int: int = NavigationMode.ARUCO.value

    # make an instance of the navigator node!
    navigator: Node = Node(
        executable="navigator_node",
        package="navigator",
        name="navigator_node",
        parameters=[
            {"longitude": longitude, "latitude": latitude, "mode": mode_int},
            {"use_sim_time": use_sim_time},
        ],
    )

    # turn on debug logs
    log_setting: DeclareLaunchArgument = DeclareLaunchArgument(
        name="log_level",
        default_value="debug",
        description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
    )

    return LaunchDescription(
        [
            SetParameter(name="use_sim_time", value=True),
            gazebo_launch_file,
            navigator,
            log_setting,
            # rviz2,
            rover_launch_file,
        ],
    )
