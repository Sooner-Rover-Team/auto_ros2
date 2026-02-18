"""
This file starts everything needed for `ros2_control`.

See its source here:
https://github.com/ros-controls/ros2_control/blob/humble/controller_manager/controller_manager/spawner.py
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_controller_manager = LaunchConfiguration("start_controller_manager")
    controllers_conf: PathJoinSubstitution = _get_controllers_conf()

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            controllers_conf,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=IfCondition(start_controller_manager),
    )

    # unfortunately, this dumb Node has to exist until we upgrade to Jazzy or
    # Kilted due to limitations in the Humble version of the
    # `controller_manager::spawner` Node.
    #
    # that's because the `--controller-ros-args` flag on that Node won't exist
    # until Jazzy! D:
    cmd_vel_remapping_relay = Node(
        name="cmdvel_relay",
        executable="relay",
        package="topic_tools",
        arguments=[
            "/cmd_vel",
            "/diff_drive_controller/cmd_vel_unstamped",
        ],
        parameters=[{"use_sim_time": True}],
    )

    # spawn the ros2_control... controllers:
    #
    # see: https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html#helper-scripts
    spawn_joint_state_broadcaster: Node = Node(
        name="joint_state_broadcaster_spawner",
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )
    spawn_diff_drive: Node = Node(
        package="controller_manager",
        name="diff_drive_controller_spawner",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_conf,
        ],
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/diff_drive_controller/cmd_vel", "/cmd_vel"),
            ("/diff_drive_controller/odom", "/odom"),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            DeclareLaunchArgument(
                "start_controller_manager",
                default_value="true",
            ),
            controller_manager,
            TimerAction(
                period=0.0,
                actions=[spawn_joint_state_broadcaster],
                condition=UnlessCondition(start_controller_manager),
            ),
            #
            # spawn controllers only when controller manager is up.
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=controller_manager,
                    on_start=[spawn_joint_state_broadcaster],
                ),
                condition=IfCondition(start_controller_manager),
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=spawn_joint_state_broadcaster,
                    on_start=[spawn_diff_drive],
                ),
                condition=IfCondition(start_controller_manager),
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=spawn_joint_state_broadcaster,
                    on_start=[spawn_diff_drive],
                ),
                condition=UnlessCondition(start_controller_manager),
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=spawn_diff_drive,
                    on_start=[cmd_vel_remapping_relay],
                )
            ),
        ]
    )


def _get_controllers_conf() -> PathJoinSubstitution:
    return PathJoinSubstitution(
        [
            FindPackageShare("drive_launcher"),
            "params",
            "ros2_control",
            "controllers.yaml",
        ]
    )
