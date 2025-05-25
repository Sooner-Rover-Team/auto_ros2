"""
This is a launch file for the mono camera.

For more information, see `/docs/hardware/sensors.md`.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import (
    Node,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Node to launch the mono camera (normal camera we will use for normal camera stuff)
    mono_camera: Node = Node(
        name="v4l2_mono_camera_image",
        package="v4l2_camera",
        executable="v4l2_camera_node",
        remappings=[
            ("/sensors/mono_image/raw", "image_raw"),
            ("/sensors/mono_image/camera_info", "camera_info"),
        ],
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("see3cam"), "params", "v4l2_camera.yaml"],
            ),
            {
                "camera_info_url": PathJoinSubstitution(
                    [
                        FindPackageShare("see3cam"),
                        "params",
                        "generated_file.yaml", # TODO: GENERATE THIS
                    ]
                )
            },
        ],
    )

    return LaunchDescription(
        [
            left_cap,
            right_cap,
            container,
        ]
    )
