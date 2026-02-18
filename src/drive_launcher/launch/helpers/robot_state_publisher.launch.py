from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    starts the `robot_state_publisher::robot_state_publisher` node.
    """
    use_sim_time = LaunchConfiguration("use_sim_time")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    use_gazebo_ros2_control = LaunchConfiguration("use_gazebo_ros2_control")

    # grab the rover's udrf
    robot_desc: dict[str, ParameterValue] = _grab_robot_description(
        ros2_control_plugin=ros2_control_plugin,
        use_gazebo_ros2_control=use_gazebo_ros2_control,
    )

    # the `robot_state_publisher` node basically takes the various transform
    # differences defined in a URDF file and tells other nodes where stuff on
    # your robot is.
    #
    # so, on the Rover, we use this for, e.g., offsetting the GPS by the
    # correct distance for consistent mapping
    rover_state_publisher_node: Node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_desc,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            DeclareLaunchArgument(
                "ros2_control_plugin",
                default_value="fake_components/GenericSystem",
            ),
            DeclareLaunchArgument(
                "use_gazebo_ros2_control",
                default_value="false",
            ),
            rover_state_publisher_node,
        ]
    )


def _grab_robot_description(
    ros2_control_plugin: LaunchConfiguration,
    use_gazebo_ros2_control: LaunchConfiguration,
) -> dict[str, ParameterValue]:
    """grabs the robot desc."""

    # make the description
    robot_description_content: Command = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("simulator"),
                    "resource",
                    "rover.urdf.xacro.xml",
                ]
            ),
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin,
            " ",
            "use_gazebo_ros2_control:=",
            use_gazebo_ros2_control,
        ]
    )

    # return it in a dict
    return {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
