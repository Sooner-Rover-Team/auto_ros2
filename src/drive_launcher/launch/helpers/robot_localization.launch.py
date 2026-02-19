from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions.find_package import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    """
    launches important `robot_localization` stuff
    """
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # the `robot_localization::navsat_transform_node` reads the gps readings
    # and converts them to the world frame scale.
    #
    # configured according to the old-ahh docs from Melodic. there are no recent
    # builds of the docs (>=humble). see:
    #
    # https://docs.ros.org/en/melodic/api/robot_localization/html/integrating_gps.html#configuring-navsat-transform-node
    navsat_transform_node: Node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform_node",
        parameters=[
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "robot_localization",
                    "navsat_transform.yaml",
                ]
            ),
            {"use_sim_time": use_sim_time},
        ],
        #
        # this node prints what it found in [INFO] every second. which is
        # annoying...
        #
        # this turns it off!
        arguments=["--ros-args", "--log-level", "warn"],
        respawn=True,
        #
        # note: these are pretty important. this node will silently do nothing
        # wihtout this (not great ergonomics lol)
        remappings=[
            ("/gps/fix", "/sensors/gps"),  # sensor_msgs::msg::NavSatFix
            ("/imu/data", "/unilidar/imu"),  # sensor_msgs::msg::Imu
            # (keep... /odometry/filtered), # sensor_msgs::msg::Odometry
        ],
    )

    # local odometry ekf filter (for imu)...
    #
    # (odom -> base_link) tf
    ekf_filter_node_odom: Node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        parameters=[
            {"use_sim_time": use_sim_time},
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "robot_localization",
                    "ekf_filter_node_odom.yaml",
                ]
            ),
        ],
        arguments=[
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "robot_localization",
                    "ekf_filter_node_odom.yaml",
                ]
            ),
        ],
        respawn=True,
        remappings=[
            ("/gps/fix", "/sensors/gps"),  # sensor_msgs::msg::NavSatFix
            ("/imu/data", "/unilidar/imu"),  # sensor_msgs::msg::Imu
            # (keep... /odometry/filtered),
        ],
    )

    # global odometry ekf filter (for gps)...
    #
    # (map -> odom) tf
    ekf_filter_node_map: Node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        parameters=[
            {"use_sim_time": use_sim_time},
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "robot_localization",
                    "ekf_filter_node_map.yaml",
                ]
            ),
        ],
        arguments=[
            PathJoinSubstitution(
                [
                    pkg_drive_launcher,
                    "params",
                    "robot_localization",
                    "ekf_filter_node_map.yaml",
                ]
            ),
        ],
        respawn=True,
        remappings=[
            ("/gps/fix", "/sensors/gps"),  # sensor_msgs::msg::NavSatFix
            ("/imu/data", "/unilidar/imu"),  # sensor_msgs::msg::Imu
            ("/odometry/filtered", "/ekf_filter_node_map/odometry/filtered"),
        ],
    )

    return LaunchDescription(
        [
            # start w/ the `use_sim_time` arg
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            # add the fr nodes
            navsat_transform_node,
            ekf_filter_node_odom,
            ekf_filter_node_map,
        ]
    )
