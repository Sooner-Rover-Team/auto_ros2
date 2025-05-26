from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions.find_package import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    pkg_zed: str = get_package_share_directory("zed_wrapper")
    pkg_simulator: str = get_package_share_directory("simulator")

    # include ZED's bundled launch file to avoid verbose nonsense.
    #
    # doing so avoids launching all the nodes ourselves
    zed_camera_launch_file: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_zed,
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
            ("publish_map_tf", "true"),
            ("publish_imu_tf", "true"),
            ("xacro_path", f"{pkg_simulator}/resource/rover.urdf.xacro.xml"),
            ("enable_gnss", "true"),
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
            # ...gps fusion sub
            SetRemap(src="/fix", dst="/sensors/gps"),
            #
            # ...depth camera stuff
            SetRemap(src="depth/depth_registered", dst="depth_registered"),
            SetRemap(src="depth/camera_info", dst="camera_info"),
            SetRemap(src="depth/depth_info", dst="depth_info"),
            #
            # and, now, add the launch file itself.
            #
            # it's ran after all the other stuff, so the remaps will apply
            zed_camera_launch_file,
        ],
    )

    return LaunchDescription([remapping_group])
