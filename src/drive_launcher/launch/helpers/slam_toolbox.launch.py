from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    starts the `slam_toolbox` mapping node (to get the `/map` topic)
    """
    use_sim_time: LaunchConfiguration = LaunchConfiguration(
        "use_sim_time", default="false"
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[
            _get_conf(),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            slam_toolbox,
        ]
    )


def _get_conf() -> PathJoinSubstitution:
    return PathJoinSubstitution(
        [
            FindPackageShare("drive_launcher"),
            "params",
            "slam_toolbox.yaml",
        ]
    )
