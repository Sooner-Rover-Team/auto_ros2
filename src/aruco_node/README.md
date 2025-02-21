# `aruco_node`

## ArUco Node

This node subscribes to a ROS Image topic, processes the images for a specified ArUco marker, and publishes the pose of the ArUco marker to a new topic.

## Image Capture Node

This node let's you specify a specific camera to capture frames from and then publishes them to a ROS Image topic.

## Extra Scripts

### ChArUco Camera Calibration

This script will let the user capture frames of a ChArUco calibration board and then use those frames to calculate a camera calibration yaml file which can be used to accurately estimate ArUco pose.

# ROS Package Dependencies
* `ros-rolling-sensor-msgs`
* `ros-rolling-geometry-msgs`
