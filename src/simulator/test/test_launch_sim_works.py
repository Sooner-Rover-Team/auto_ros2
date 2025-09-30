"""
this integration-tests launch files using the `launch_testing` package.

for more info about that, please see:
https://arnebaeyens.com/blog/2024/ros2-integration-testing/
"""

import unittest

import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch_testing import IoHandler, ProcInfoHandler
from launch_testing.actions import ReadyToTest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
)


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description() -> LaunchDescription:
    pkg_simulator: str = get_package_share_directory("simulator")

    # launch gazebo stuff only
    gazebo_launch_file: IncludeLaunchDescription = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        pkg_simulator,
                        "launch",
                        "gazebo_only.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments=[
            # use sim time
            ("use_sim_time", "True"),
            # run without showing a window
            ("run_headless", "True"),
            # ...but don't actually simulate anything
            ("run_sim_immediately", "False"),
            # and use sim time
            ("use_sim_time", "True"),
        ],
    )

    # test start condition (wait a sec for everything to launch)
    start_cond: TimerAction = TimerAction(period=1.5, actions=[ReadyToTest()])

    return LaunchDescription([gazebo_launch_file, start_cond])


class TestGazeboLaunch(unittest.TestCase):
    """ensures that all expected nodes are running (and they don't fail)"""

    def test_nodes_running(self, proc_info: ProcInfoHandler):
        """checks that all our expected nodes are running"""
        expected_nodes = [
            "create",  # rover model spawner
            "parameter_bridge",  # (gz) bridge
            "image_bridge",  # (gz camera) bridge
            "simulator",  # (soro) bridge
            # note: we actually skip gazebo, as it can run forever
        ]

        for node in expected_nodes:
            proc_info.assertWaitForStartup(process=node, timeout=5.0)  # pyright: ignore[reportAttributeAccessIssue]

    def test_no_process_errors(self, proc_output: IoHandler):
        """ensures nothing err'd out"""
        for process in proc_output.processes():
            output: str = process.output
            process_name: str = process.name

            for line in output:
                assert "error" not in line.lower(), (
                    f"`{process_name}` failed to start! err: {line}"
                )


'''
TODO(bray): get this working when we've got time.
    - gazebo will sometimes be really slow to stop
    - this asserts that it exits in time
    - ...so there's your problem

@launch_testing.post_shutdown_test()
class TestGazeboShutdown(unittest.TestCase):
    """checks that the shutdown works right"""

    def test_exit_codes(self, proc_info: ProcInfoHandler):
        """ensure all nodes exit successfully"""
        for process_info in proc_info:
            if process_info.returncode is not None:
                assert process_info.returncode == 0, (
                    f"`{process_info.process_name}` had a bad exit code: `{process_info.returncode}`"
                )
'''
