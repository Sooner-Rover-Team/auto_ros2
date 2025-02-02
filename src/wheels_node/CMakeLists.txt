cmake_minimum_required(VERSION 3.5)
project(wheels_node)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/Wheels.msg"
)


rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
)

ament_export_dependencies(rosidk_default_runtime)
ament_package()
