# this launch file handles launching Nav2 in the right order, with support for
# GPS coordinate mapping.

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # whether or not we'll launch using sim time.
    use_sim_time = LaunchConfiguration("use_sim_time")

    # start the utm conversion node.
    #
    # it allows us to provide GNSS coordinates to navigate to. wtihout it, we
    # have no frame of reference onto the map.
    utm_conversion_node: Node = Node(
        executable="utm_conversion_node",
        package="navigator",
        name="utm_conversion_node",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # this collection of nodes is from the `robot_localization` package. these
    # provide essential functionality for the Rover.
    #
    # together, they work to translate known positions from our (hopefully
    # accurate) URDF model of the Rover into offsets.
    #
    # `robot_localization::navsat_transform_node` offsets the recv'd GPS
    # coordinates such that they are local to the Rover's origin.
    #
    # next up, we have two `robot_localization::ekf_node` instances...
    #
    # the first is `ekf_filter_node_odom`, which reads from the IMU and
    # converts its data into something usable on the frame (in other words,
    # local "odometry" data). other nodes use that info for state estimation!
    #
    # on the other hand, we've got `ekf_filter_node_map`. it uses info from the
    # `navsat_transform_node` and `ekf_filter_node_odom` to make a "global"
    # representation of the map. it's used by the planners (and, as a result,
    # directly by our Navigator).
    robot_localization: IncludeLaunchDescription = _robot_localization(use_sim_time)

    # the `depthimage_to_laserscan::depthimage_to_laserscan_node` gives us
    # another source of `/tf:map` data.
    #
    # it converts our depth camera data (from the Zed 2i) into usable point
    # clouds, which you'd otherwise have to get from a laser scanner.
    #
    # our Rover doesn't have one of those..! so, for object avoidance, we use
    # this node to add boundaries to our map.
    depthimage_to_laserscan: IncludeLaunchDescription = _depthimage_to_laserscan(
        use_sim_time
    )

    # ok, so this part launches a few different things...
    #
    # first, we start some nodes from `slam_toolbox`, all of which provide
    # SLAM tracking to the Rover. importantly, **this node provides the
    # mandatory `map -> odom -> base_link` frame chain for Nav to start doing
    # anything useful.
    #
    # after that, we start a Node (the `navigator::navigation_bringup_node`) to
    # wait on the `slam_toolbox` to make those links properly.
    #
    # finally, we launch Nav2! it provides navigation utilties to the SoRo
    # Navigator and beyond.
    #
    # it's primarily helpful for its provided algorithms that are really
    # difficult (and verbose) to implement by hand. you control it through the
    # various actions its plugins provide during navigation.
    #
    # in any case, here's that launch config...
    navigation_stack: IncludeLaunchDescription = _navigation_bringup(use_sim_time)

    # we'll also want the `robot_state_publisher::robot_state_publisher` node.
    #
    # it says where things are on the Rover in relation to one another, which
    # is required for consistent mapping, good navigation, and object
    # avoidance.
    robot_state_publisher: IncludeLaunchDescription = _robot_state_publisher(
        use_sim_time
    )

    # `ros2_control` is a collection of nodes that we use to control the Rover.
    #
    # it recvs instructions from any node, but in practice, it'll always come
    # directly from the Nav2 stack directing the robot where to go (and how not
    # to get stuck on giant cliff boulders).
    #
    # the nodes from this package are particularly useful in simulating and
    # visualizing the Rover, as they provide simple `tf2` transforms on the
    # wheels as they spin.
    #
    # we currently do not use this package for directly manipulating the
    # wheels, as doing so would require writing a `ros2_control` hardware
    # plugin.
    #
    # that'd be an extremely painful task with our microcontroller setup, as
    # there are no cross-platform networking APIs in the C++ stdlib. note that
    # `boost` could be an option in the future if future Autonomous members9
    # would like to move toward a `ros2_control`-based approach.
    ros2_control: IncludeLaunchDescription = _ros2_control(use_sim_time)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            utm_conversion_node,
            robot_state_publisher,
            robot_localization,
            depthimage_to_laserscan,
            navigation_stack,
            ros2_control,
            Node(
                executable="aruco_node",
                package="aruco",
                name="aruco_node",
            ),
        ]
    )


def _robot_localization(
    use_sim_time: LaunchConfiguration,
) -> IncludeLaunchDescription:
   pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "robot_localization.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[("use_sim_time", (use_sim_time))],
    )


def _robot_state_publisher(
    use_sim_time: LaunchConfiguration,
) -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "robot_state_publisher.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[("use_sim_time", (use_sim_time))],
    )


def _navigation_bringup(
    use_sim_time: LaunchConfiguration,
) -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "navigation_stack.launch.py",
                    ]
                )
            ],
        ),
        launch_arguments=[("use_sim_time", (use_sim_time))],
    )


def _depthimage_to_laserscan(
    use_sim_time: LaunchConfiguration,
) -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "depthimage_to_laserscan.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[("use_sim_time", use_sim_time)],
    )


def _ros2_control(
    use_sim_time: LaunchConfiguration,
) -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "ros2_control.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[("use_sim_time", use_sim_time)],
    )
