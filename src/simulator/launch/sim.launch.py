from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.actions import Node, SetParameter
from navigator_node.types import NavigationMode


def generate_launch_description():
    pkg_simulator: str = get_package_share_directory("simulator")

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
        )
    )

    # launch rviz2
    rviz2: Node = Node(
        package="rviz2",
        executable="rviz2",
    )

    # add some coordinate to go to...
    latitude = 0.0002
    longitude = 0.0002

    mode_int: int = NavigationMode.ARUCO.value

    nav2_bringup_node: Node = Node(
        name="nav2_bringup_node",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
    )

    # start nav2 using our bringup script
    nav2_bringup_dir: str = get_package_share_directory("drive_launcher")
    nav2_bringup: LaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, "/launch", "/helpers", "/start_nav2.py"],
        ),
        launch_arguments={
            "use_sim_time": "True",
            "namespace": "",
            "container_name": "nav2_bringup_node",
            "params_file": PathJoinSubstitution([pkg_simulator, "params", "nav2.yaml"]),
            "autostart": "True",
        }.items(),
    )

    # make an instance of the navigator node!
    navigator: Node = Node(
        executable="navigator_node",
        package="navigator_node",
        name="navigator",
        parameters=[{"longitude": longitude, "latitude": latitude, "mode": mode_int}],
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
            nav2_bringup_node,
            nav2_bringup,
            rviz2,
        ],
    )
