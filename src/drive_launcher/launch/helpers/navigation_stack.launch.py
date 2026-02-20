from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.actions.node import ExecuteProcess


def generate_launch_description() -> LaunchDescription:
    """
    here, we combine the launch file from `slam_toolbox`, a node to watch
    progress on a non-zero map while moving the Rover backwards SLOWWWWWLY, and
    Nav2 itself, waiting on the map-watcher Node to complete its work.

    let's explain a bit more about why...

    before making these changes, our Nodes all launched together... which is
    not okay for `Nav2`. you can see it complaining below:

    ```console
    [component_container-15] [WARN] [1746053782.679414748] [local_costmap.local_costmap]: Can't update static costmap layer, no map received
    [navsat_transform_node-9] [INFO] [1746053783.109746597] [navsat_transform_node]: Datum (latitude, longitude, altitude) is (0.00, 0.00, 0.18)
    [navsat_transform_node-9] [INFO] [1746053783.109789527] [navsat_transform_node]: Datum UTM coordinate is (31N, 166021.74, 0.00)
    [component_container-15] [ERROR] [1746053783.192772312] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
    [component_container-15] [ERROR] [1746053783.192777842] [local_costmap.local_costmap]: Received map message is malformed. Rejecting.
    [component_container-15] [WARN] [1746053783.679459721] [local_costmap.local_costmap]: Can't update static costmap layer, no map received
    [navsat_transform_node-9] [INFO] [1746053784.131221008] [navsat_transform_node]: Datum (latitude, longitude, altitude) is (-0.00, 0.00, 0.18)
    [navsat_transform_node-9] [INFO] [1746053784.131277188] [navsat_transform_node]: Datum UTM coordinate is (31M, 166021.74, 10000000.00)
    [component_container-15] [ERROR] [1746053784.192873064] [local_costmap.local_costmap]: Received map message is malformed. Rejecting.
    [component_container-15] [ERROR] [1746053784.192879724] [global_costmap.global_costmap]: Received map message is malformed. Rejecting.
    ```

    well, run `ros2 topic echo /map --once`, and we can see what caused those
    errors:

    ```yaml
    header:
      stamp:
        sec: 4
        nanosec: 0
      frame_id: map
    info:
      map_load_time:
        sec: 0
        nanosec: 0
      resolution: 0.05000000074505806
      width: 0
      height: 0
      origin:
        position:
          x: 0.3
          y: 0.0
          z: 0.0
        orientation:
          x: 0.0
          y: 0.0
          z: 0.0
          w: 1.0
    data: []
    ---
    ```

    ...do you see it? look closer:

    ```yaml
    width: 0
    height: 0
    ```

    that's a map with no size! when `slam_toolbox` launches, it makes a "blank"
    map until it can get more info about the environment. more importantly, it
    needs more information about the environment before it can assume anything.

    so, we make the Rover move before starting Nav2. that's what this file is
    about. sorry for the verbose explanation lol
    """
    use_sim_time: LaunchConfiguration = LaunchConfiguration("use_sim_time")

    # the `slam_toolbox::async_slam_toolbox_node` fills the `/tf:map` and
    # `tf:odom` frames with our translated laserscan data.
    slam_toolbox: ExecuteProcess = _slam_toolbox(use_sim_time)
    delayed_slam_toolbox: TimerAction = TimerAction(
        period=2.5,
        actions=[slam_toolbox],
    )

    navigation_bringup_node: Node = Node(
        package="navigator",
        executable="navigation_bringup_node",
        name="navigation_bringup_node",
        shell=True,
        output="screen",
    )

    nav2: IncludeLaunchDescription = _nav2(use_sim_time)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
            ),
            SetParameter("use_sim_time", use_sim_time),
            #
            # first, the toolbox
            delayed_slam_toolbox,
            #
            # after that, we can launch the lil watcher node
            RegisterEventHandler(
                OnProcessStart(
                    target_action=slam_toolbox,
                    on_start=[navigation_bringup_node],
                )
            ),
            #
            # then, when that exits, we can finally start nav2
            RegisterEventHandler(
                OnProcessExit(
                    target_action=navigation_bringup_node,
                    on_exit=nav2,
                )
            ),
        ]
    )


# def _slam_toolbox(
#     use_sim_time: LaunchConfiguration,
# ) -> IncludeLaunchDescription:
def _slam_toolbox(
    use_sim_time: LaunchConfiguration,
) -> ExecuteProcess:
    # pkg_drive_launcher: str = get_package_share_directory("drive_launcher")
    # return IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             PathJoinSubstitution(
    #                 [
    #                     pkg_drive_launcher,
    #                     "launch",
    #                     "helpers",
    #                     "slam_toolbox.launch.py",
    #                 ]
    #             )
    #         ],
    #     ),
    #     launch_arguments=[("use_sim_time", (use_sim_time))],
    # )
    #
    # FIXME(bray): this is a ridiculous workaround - please never use it again.
    #
    # if you have any knowledge about how to solve this a better way, please
    # send in a PR!
    #
    # stack overflow post (by bray):
    # https://robotics.stackexchange.com/questions/115505/on-ros-2-humble-can-you-add-a-launch-file-into-a-registereventhandleronprocess
    return ExecuteProcess(
        name="slam_toolbox_start_node",
        cmd=[
            "ros2",
            "launch",
            "drive_launcher",
            "slam_toolbox.launch.py",
            # 😮‍💨 yet another hack...
            # quote launch config value so PythonExpression doesn't evaluate
            # `false`/`true` as Python names.
            PythonExpression(["'use_sim_time:=' + str('", use_sim_time, "')"]),
        ],
        shell=True,
        output="screen",
    )


def _nav2(
    use_sim_time: LaunchConfiguration,
) -> IncludeLaunchDescription:
    pkg_drive_launcher: str = get_package_share_directory("drive_launcher")

    # start Nav2 with our helper script
    nav2_launch: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_drive_launcher,
                        "launch",
                        "helpers",
                        "nav2.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "namespace": "",
            "container_name": "nav2_container",
        }.items(),
    )

    return nav2_launch
