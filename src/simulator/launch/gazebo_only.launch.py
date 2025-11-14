import os

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
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_simulator: str = get_package_share_directory("simulator")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # allow parent launch files to ask us to run headless
    run_headless: LaunchConfiguration = LaunchConfiguration("run_headless")
    run_sim_immediately: LaunchConfiguration = LaunchConfiguration(
        "run_sim_immediately"
    )

    # same as above, except we're mooching off it and also its for joints
    rover_joint_publisher_node: Node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # spawn the rover model
    rover_model_spawner: Node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic",
            "/robot_description",
            "-name",
            "remi",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.2",
        ],
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # bridge for ros 2 <---> gazebo topics, minus cameras
    bridge_params: str = os.path.join(
        pkg_simulator,
        "params",
        "bridge.yaml",
    )
    bridge: Node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        parameters=[
            os.path.join(pkg_simulator, "params", "camera.yaml"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # bridge node for the cameras...
    camera_bridge: Node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/sensors/mono_image", "/sensors/depth_image"],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            },
        ],
        output="screen",  # print any logs to terminal
    )

    # a custom bridge to use our message types.
    #
    # yea... it's goin through three layers lol
    soro_bridge: Node = Node(
        package="simulator",
        executable="simulator",
        parameters=[
            {"use_sim_time": use_sim_time},
        ],
    )

    # spawn the gazebo server
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_launch_path = PathJoinSubstitution(
        [pkg_ros_gz_sim, "launch", "gz_sim.launch.py"]
    )
    gz_server: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution([pkg_simulator, "resource", "world.sdf.xml"]),
                PythonExpression([run_sim_immediately, " and ' -r' or ''"]),
                PythonExpression([run_headless, " and ' -s' or ''"]),
            ],
            "on_exit_shutdown": "True",
            "gz_version": "8",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            # SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1"),
            DeclareLaunchArgument(
                name="run_headless",
                default_value="False",
                description="ask Gazebo to start and run, but not render itself on a surface",
            ),
            DeclareLaunchArgument(
                name="run_sim_immediately",
                default_value="True",
                description="whether Gazebo will start running immediately. defaults to running immediately.",
            ),
            bridge,
            camera_bridge,
            gz_server,
            soro_bridge,
            rover_joint_publisher_node,
            rover_model_spawner,
        ],
    )
