"""
This is a launch file for the Zed 2i depth camera.

For more information, see `/docs/hardware/sensors.md`.
"""

from launch import LaunchDescription
from launch.frontend.parse_substitution import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import (
    Node,
)
from launch_ros.actions.load_composable_nodes import (
    ComposableNode,
    ComposableNodeContainer,
)
from launch_ros.substitutions import FindPackageShare

_CONTAINER_NAME: str = "zed_conversion_container"


def generate_launch_description() -> LaunchDescription:
    zed_package_path: str = get_package_share_directory("zed")

    # these two nodes capture images from the left + right camera respectively.
    #
    # it's separated like that on the hardware, so we do need *two* nodes.
    # however, another node will stich them together for a disparity output
    # later on...
    left_cap: Node = Node(
        name="v4l2_depth_image_left",
        package="v4l2_camera",
        executable="v4l2_camera_node",
        remappings=[
            ("image_raw", "/sensors/depth_image/left_image_raw"),
            ("camera_info", "/sensors/depth_image/left_camera_info"),
        ],
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("zed"), "params", "v4l2_camera.yaml"]
            ),
            {
                "camera_info_url": f"file://{zed_package_path}/params/left_camera_info_hd.yaml"
            },
        ],
    )
    right_cap: Node = Node(
        name="v4l2_depth_image_right",
        package="v4l2_camera",
        executable="v4l2_camera_node",
        remappings=[
            ("image_raw", "/sensors/depth_image/right_image_raw"),
            ("camera_info", "/sensors/depth_image/right_camera_info"),
        ],
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("zed"), "params", "v4l2_camera.yaml"],
            ),
            {
                "camera_info_url": f"file://{zed_package_path}/params/right_camera_info_hd.yaml"
            },
        ],
    )

    # we'll also have two "rectify" nodes to correct our cameras' distortions.
    #
    # these run on the left and right (as previously shown)
    left_rectifier: ComposableNode = ComposableNode(
        name="rectify_depth_image_left",
        package="image_proc",
        plugin="image_proc::RectifyNode",
        remappings=[
            ("image", "/sensors/depth_image/left_image_raw"),
            ("camera_info", "/sensors/depth_image/left_camera_info"),
            ("image_rect", "/sensors/depth_image/left_image_rect"),
        ],
    )
    right_rectifier: ComposableNode = ComposableNode(
        name="rectify_depth_image_right",
        package="image_proc",
        plugin="image_proc::RectifyNode",
        remappings=[
            ("image", "/sensors/depth_image/right_image_raw"),
            ("camera_info", "/sensors/depth_image/right_camera_info"),
            ("image_rect", "/sensors/depth_image/right_image_rect"),
        ],
    )

    # this node provides an initial stitching of the two images together, but
    # it won't yet be a depth mapping.
    #
    # we'll need the next node for that. just know that doing this prepares us
    # to get an output PointCloud2 :D
    #
    # note: docs are only available for `rolling` for some reason, not humble.
    # you can find them here:
    # https://docs.ros.org/en/rolling/p/stereo_image_proc/doc/components.html
    stitch_disparity: ComposableNode = ComposableNode(
        name="disparity_depth_image",
        package="stereo_image_proc",
        plugin="stereo_image_proc::DisparityNode",
        remappings=[
            # the node subscribes to these topics
            ("left/camera_info", "/sensors/depth_image/left_camera_info"),
            ("left/image_rect", "/sensors/depth_image/left_image_rect"),
            ("right/camera_info", "/sensors/depth_image/right_camera_info"),
            ("right/image_rect", "/sensors/depth_image/right_image_rect"),
            #
            # it publishes the disparity onto this topic
            ("disparity", "/sensors/depth_image/disparity"),
        ],
    )

    # ok; this is the next node! it provides the PointCloud2 for Nav2 to create
    # good costmaps (REQUIRED for object avoidance)
    stich_point_cloud: ComposableNode = ComposableNode(
        name="point_cloud_depth_image",
        package="stereo_image_proc",
        plugin="stereo_image_proc::PointCloudNode",
        remappings=[
            # subscriptions
            ("disparity", "/sensors/depth_image/disparity"),
            ("left/camera_info", "/sensors/depth_image/left_camera_info"),
            ("left/image_rect_color", "/sensors/depth_image/left_image_rect"),
            ("right/camera_info", "/sensors/depth_image/right_camera_info"),
            (
                "right/image_rect_color",
                "/sensors/depth_image/right_image_rect",
            ),
            # it publishes the point cloud onto this topic
            ("points2", "/sensors/depth_image/point_cloud"),
        ],
    )

    # finally, we can create a LaserScan for each PointCloud2
    pointcloud_to_laserscan: ComposableNode = ComposableNode(
        name="pointcloud_to_laserscan",
        package="pointcloud_to_laserscan",
        plugin="pointcloud_to_laserscan::PointCloudToLaserScanNode",
        remappings=[
            ("/sensors/depth_image/point_cloud", "cloud_in"),
            ("/scan", "scan"),
        ],
    )

    composable_nodes: list[ComposableNode] = [
        left_rectifier,
        right_rectifier,
        stitch_disparity,
        stich_point_cloud,
        pointcloud_to_laserscan,
    ]

    container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return LaunchDescription(
        [
            left_cap,
            right_cap,
            container,
        ]
    )
