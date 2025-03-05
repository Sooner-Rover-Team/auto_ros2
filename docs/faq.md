# FAQ

Some issues we've experienced and potential solutions.

## ROS 2 randomly blew up!

You were just screwing around and ROS 2 blew up in your face! Maybe it won't even talk to the DDS at this point?

This can happen if you `killall python3`, which messes with ROS 2:

```python3
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 topic list
Traceback (most recent call last):
  File "/opt/ros/humble/bin/ros2", line 33, in <module>
    sys.exit(load_entry_point('ros2cli==0.18.11', 'console_scripts', 'ros2')())
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2cli/cli.py", line 91, in main
    rc = extension.main(parser=parser, args=args)
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2topic/command/topic.py", line 41, in main
    return extension.main(args=args)
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2topic/verb/list.py", line 56, in main
    topic_names_and_types = get_topic_names_and_types(
  File "/opt/ros/humble/lib/python3.10/site-packages/ros2topic/api/__init__.py", line 41, in get_topic_names_and_types
    topic_names_and_types = node.get_topic_names_and_types()
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1122, in __call__
    return self.__send(self.__name, args)
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1464, in __request
    response = self.__transport.request(
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1166, in request
    return self.single_request(host, handler, request_body, verbose)
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1182, in single_request
    return self.parse_response(resp)
  File "/usr/lib/python3.10/xmlrpc/client.py", line 1354, in parse_response
    return u.close()
  File "/usr/lib/python3.10/xmlrpc/client.py", line 668, in close
    raise Fault(**self._stack[0])
xmlrpc.client.Fault: <Fault 1: "<class 'RuntimeError'>:!rclpy.ok()">
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main) [1]> ros2 daemon stop
The daemon has been stopped
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 daemon start
The daemon has been started
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 topic list
/control/wheels
```

To fix this, restart its daemon: `ros2 daemon stop && ros2 daemon start`

## After a clean build, it's saying stuff has no metadata!

If you do a clean build, but only use `colcon build` (as opposed to `colcon build --symlink-install`), you're going to get this kind of error:

```
importlib_metadata.PackageNotFoundError: No package metadata was found for ros2cli
```

Also, make sure to refresh your environment afterwards by re-sourcing the `local_install.bash` file. `. ./SOURCE_SCRIPT.fish` works well for this! :)

## After a clean build, `ros2 run` says "no executable found"!

```fish
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> rm -rfd install/ build/ log/
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> colcon build --packages-select navigator_node
Starting >>> navigator_node
Finished <<< navigator_node [0.74s]

Summary: 1 package finished [0.94s]
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 run navigator_node navigator_node
No executable found
```

Like above, you need to use `colcon build --symlink-install`.

## `ros2 launch` gives "libexec directory does not exist"

```fish
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main)> ros2 launch simulator sim.launch.py
[INFO] [launch]: All log files can be found below /home/barrett/.ros/log/2025-02-25-01-25-56-666083-sls2-52704
[INFO] [launch]: Default logging verbosity is set to INFO
[ERROR] [launch]: Caught exception in launch (see debug for traceback): package 'navigator_node' found at '/home/barrett/Documents/projects/Rover/auto_ros2/install/navigator_node', but libexec directory '/home/barrett/Documents/projects/Rover/auto_ros2/install/navigator_node/lib/navigator_node' does not exist
(autonomous) barrett@sls2 ~/D/p/R/auto_ros2 (main) [1]>
```

Use `--symlink-install`!

## At runtime, I got a `TypeError`... Unhashable type?

```python3
[simulator-4] 2025-02-25 01:29:30.844 | INFO     | simulator.main:main:127 - starting simulator driver...
[simulator-4] Traceback (most recent call last):
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 224, in spin
[simulator-4]     executor.add_node(node)
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 254, in add_node
[simulator-4]     if node not in self._nodes:
[simulator-4] TypeError: unhashable type: 'SoroBridge'
[simulator-4]
[simulator-4] During handling of the above exception, another exception occurred:
[simulator-4]
[simulator-4] Traceback (most recent call last):
[simulator-4]   File "/home/barrett/Documents/projects/Rover/auto_ros2/install/simulator/lib/simulator/simulator", line 33, in <module>
[simulator-4]     sys.exit(load_entry_point('simulator', 'console_scripts', 'simulator')())
[simulator-4]   File "/home/barrett/Documents/projects/Rover/auto_ros2/build/simulator/simulator/main.py", line 139, in main
[simulator-4]     rclpy.spin(bridge_node)
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py", line 228, in spin
[simulator-4]     executor.remove_node(node)
[simulator-4]   File "/opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/executors.py", line 270, in remove_node
[simulator-4]     self._nodes.remove(node)
[simulator-4] TypeError: unhashable type: 'SoroBridge'
```

Implement Hash for your type:

```python3
from dataclasses import dataclass
from typing_extensions import override

from rclpy.node import Node

@dataclass
class MyClass(Node):
    def __init__():
        super().__init__("my_class")

    @override
    def __hash__(self) -> int:
        return super().__hash__()
```
