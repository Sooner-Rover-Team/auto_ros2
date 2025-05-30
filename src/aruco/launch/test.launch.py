from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # Publish images from a video capture device
    image_capture_node = Node(
        package="aruco",
        executable="image_capture",
    )

    # Set the static pose of the video capture device
    static_transform_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        # Transform (x, y, z) Rotation (roll, yaw, pitch) parent_frame child_frame
        arguments=["1", "0", "0", "0", "0", "0", "map", "camera"],
    )
    # Start tracking aruco markers
    aruco_tracking_node = Node(
        package="aruco",
        executable="aruco_node",
    )

    ld.add_action(image_capture_node)
    ld.add_action(static_transform_node)
    ld.add_action(aruco_tracking_node)
    return ld
