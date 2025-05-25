## this file is based off the `ros2_rust_template` repository. thank you!
##
## github.com/uhobeike/ros2_rust_template/tree/main/launcher

cmake_minimum_required(VERSION 3.8)
project(zed)

# find deps
find_package(ament_cmake_auto REQUIRED)

# automatically package ourselves.
ament_auto_package(
  INSTALL_TO_SHARE
  launch
  params
)
