# `drive_launcher`

This package contains launch files for driving the Rover and starting dependencies like Nav2, SLAM, and other necessary stuff.

## Usage

TODO.

<!--
TODO: we'll need a small launch script for competition. probably a new script,
`drive.launch.py` (or something like that)
-->

## Notes on `tf2` Frames

We have several inter-dependent nodes operating within this package's launch files. However, it's **extremely important** to ensure that all `tf2` frames have exactly ONE WRITER. Otherwise, the system will display erratic behavior, and Autonomous won't go well... D:

With that in mind, here's a list of our `tf2` frames and their broadcasters:

- `base_link`, `chassis`, and all subframes are provided and chained together by the `robot_state_publisher` node.
  - `base_link` is the Rover's highest-level frame. In other words, it exists below the odometry and map frames.
- `odom` is provided by `robot_localization::ekf_node`, which we call `ekf_filter_node_odom`.
  - This frame is the Rover's [odometry frame](https://en.wikipedia.org/wiki/Odometry). We use it to track how the Rover moves over time.
  - It's vital for Nav2 to properly "localize" the Rover on the map.
- `map` is created by `slam_toolbox::async_slam_toolbox_node`.
  - The "map frame" is literally the entire world. The Rover moves within this frame.
  - You can think of it as the lowest layer in the "frame stack" - it's the first one in the tree.
  - Some diagnostics, errors, and websites will refer to this as the "root node."

### Nav2's Costmap Frames

Other important frames to note are the `local_costmap` and `global_costmap`, both of which are provided by Nav2's costmap node. They contain "costmap" data (which refers to the "cost" of moving to a specific location). These allow the Rover to find the best path to its goals, at least according to Nav2.

If nothing else, know that these frames both use a `rolling_window`, meaning they move with the Rover on the `/tf:map` frame. **Rolling costmaps are required for navigation on a large scale**.

### Troubleshooting

It's pretty difficult to find out which node is broadcasting a frame. In the absence of debugging data, you can use the following methods to check the system's status:

- Check which nodes are publishing to the topic: `ros2 topic info /tf --verbose --no-daemon | grep -e "Node name: " -e "count: "`
- Watch the `/diagnostics` topic for info on the problem: `ros2 topic echo /diagnostics --no-daemon`
- Ensure the frame chain looks alright: `ros2 run tf2_tools view_frames`
- Look for weird broadcasts between two frames: `ros2 run tf2_ros tf2_monitor map odom 20`
- Peek at the launch file logs for any `DEBUG`-level diagnostics: `cd ~/.ros/log/` and start `echo`ing lol
