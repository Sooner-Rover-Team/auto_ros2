# `aruco`

A package related to finding ArUco markers.

## `aruco::aruco_node`

This node starts an action server, subscribes to a ROS Image topic, processes each frame to find any ArUco markers, then publishes their poses as feedback.

## Image Capture Node

This node lets you specify a specific camera to capture frames from, then publishes them to a ROS 2 Image topic.

## Extra Scripts

### ChArUco Camera Calibration

This script lets the user capture frames of a ChArUco calibration board. Then, it uses those frames to calculate a camera calibration `.yaml` file. This file is used to accurately estimate ArUco pose, so it's important we get it right!
