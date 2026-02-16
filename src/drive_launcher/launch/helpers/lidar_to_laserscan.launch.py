from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    use_sim_time: LaunchConfiguration = LaunchConfiguration(
        "use_sim_time", default="false"
    )

    pointcloud_to_laserscan: ComposableNode = ComposableNode(
        name="lidar_pointcloud_to_laserscan",
        package="pointcloud_to_laserscan",
        plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
        remappings=[
            ("cloud_in", "/unilidar/cloud"),
            ("scan", "/scan"),
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "target_frame": "base_link",
                "transform_tolerance": 0.2,
                "min_height": -0.5,
                "max_height": 2.5,
                "angle_min": -3.1415926535,
                "angle_max": 3.1415926535,
                "angle_increment": 0.0058,
                "scan_time": 0.1,
                "range_min": 0.3,
                "range_max": 100.0,
                "use_inf": True,
            }
        ],
    )

    container: ComposableNodeContainer = ComposableNodeContainer(
        name="lidar_scan_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[pointcloud_to_laserscan],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            container,
        ]
    )
