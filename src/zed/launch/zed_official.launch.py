from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions.find_package import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_zed: str = get_package_share_directory("zed")
    pkg_simulator: str = get_package_share_directory("simulator")

    # include ZED's bundled launch file to avoid verbose nonsense.
    #
    # doing so avoids launching all the nodes ourselves
    zed_camera_launch_file: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        get_package_share_directory("zed_wrapper"),
                        "launch",
                        "zed_camera.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            ("camera_model", "zed2i"),
            #
            # set namespace to align w/ our original spec
            ("namespace", "/sensors/depth_image"),
            #
            # let other nodes take care of publishing to the tf tree
            # ("publish_map_tf", "false"),
            # ("publish_imu_tf", "false"),
            #
            # provide the correct path to the Rover URDF (3d model)
            # ("xacro_path", f"{pkg_simulator}/resource/rover.urdf.xacro.xml"),
            #
            # configure the wrapper to use our sensors while mapping. this can
            # increase accuracy and help in other subtle ways
            ("enable_gnss", "true"),
            #
            # use config file with QoS overrides for GPS topic compatibility
            ("ros_params_override_path", f"{get_package_share_directory('zed_wrapper')}/config/common_stereo.yaml"),
        ],
    )

    # then, we'll put it within a "group action."
    #
    # that allows us to add topic remappings for all the nodes in the ZED
    # launch file. which means we don't have a bunch of `/zed` topics randomly
    # in our `/` namespace :P
    remapping_group: GroupAction = GroupAction(
        actions=[
            # add remappings. for more info about these, please see the ZED
            # documentation:
            # https://web.archive.org/web/20240522090026/https://www.stereolabs.com/docs/ros2/zed-node
            #
            # ...sensor fusion + odometry
            SetRemap(src="/fix", dst="/sensors/gps"),
            SetRemap(src="zed/imu/data", dst="/sensors/depth_image/imu"),
            SetRemap(src="zed/odom", dst="/sensors/depth_image/odom"),
            #
            # ...depth camera stuff
            SetRemap(src="depth/depth_registered", dst="depth_registered"),
            SetRemap(src="depth/camera_info", dst="camera_info"),
            SetRemap(
                src="depth/depth_registered",
                dst="/sensors/depth_image/depth_registered",
            ),
            SetRemap(
                src="depth/camera_info", dst="/sensors/depth_image/camera_info"
            ),
            #
            # ...pointcloud stuff
            SetRemap(src="point_cloud", dst="/sensors/depth_image/point_cloud"),
            #
            # and, now, add the launch file itself.
            #
            # it's ran after all the other stuff, so the remaps will apply
            zed_camera_launch_file,
        ],
    )

    # we'll also add a node that just sticks the Zed camera (and all its many,
    # many frames) onto the chassis.
    #
    # otherwise, it's kinda just... floating there
    static_transform_node: Node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            # transform (x, y, z)
            "0",
            "0",
            "0",
            #
            # rotation (roll, yaw, pitch)
            "0",
            "0",
            "0",
            #
            # parent_frame
            "chassis",
            #
            # child_frame
            "zed_camera_link",
        ],
    )

    return LaunchDescription([remapping_group, static_transform_node])
