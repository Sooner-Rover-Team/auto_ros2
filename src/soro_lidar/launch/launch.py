from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_rviz = LaunchConfiguration("launch_rviz")

    # Run unitree lidar
    node1 = Node(
        package="soro_lidar",
        executable="unitree_lidar_ros2_node",
        name="unitree_lidar_ros2_node",
        output="screen",
        parameters=[
            {"initialize_type": 2},
            {"work_mode": 0},
            {"use_system_timestamp": True},
            {"range_min": 0.0},
            {"range_max": 100.0},
            {"cloud_scan_num": 18},
            {"serial_port": "/dev/ttyACM0"},
            {"baudrate": 4000000},
            {"lidar_port": 6101},
            {"lidar_ip": "192.168.1.62"},
            {"local_port": 6201},
            {"local_ip": "192.168.1.2"},
            {"cloud_frame": "unilidar_lidar"},
            {"cloud_topic": "unilidar/cloud"},
            {"imu_frame": "unilidar_imu"},
            {"imu_topic": "unilidar/imu"},
        ],
    )

    # Run RViz only when explicitly requested.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([FindPackageShare("soro_lidar"), "rviz", "view.rviz"]),
        ],
        output="log",
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            node1,
            rviz_node,
        ]
    )
