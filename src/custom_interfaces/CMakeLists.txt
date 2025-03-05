cmake_minimum_required(VERSION 3.8)
project(custom_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# make a list of all msg files in `msg/`.
#
# for more info, see IntelligentSystemsLabUTC/ros2-examples on GitHub!
set(MSG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSG_DIR}/*.msg")

# like the above, but for services
set(SRV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
file(GLOB SRV_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRV_DIR}/*.srv")

# and actions
set(ACT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/action")
file(GLOB ACT_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${ACT_DIR}/*.action")

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES} # generate interfaces for all message files
  ${SRV_FILES} # service files
  ${ACT_FILES} # and action files
  DEPENDENCIES geometry_msgs
  DEPENDENCIES geographic_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
