cmake_minimum_required(VERSION 3.8)
project(simulator)

# build the package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_python REQUIRED)

# install a bunch of stuff
install(DIRECTORY
    launch #...launch files
    resource # ...resources (used for sim models/etc.)
    params # ...launch parameter files
    DESTINATION share/${PROJECT_NAME}/
)

ament_python_install_package(${PROJECT_NAME})

# add a script to act like a binary for the cmake build
install(PROGRAMS
  scripts/simulator
  scripts/save_image
  DESTINATION lib/${PROJECT_NAME}
)

# and make that script executable
#
# (yes this is insane, shoutout copilot for matching my freak)
install(CODE "execute_process(COMMAND chmod +x \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/simulator)")
install(CODE "execute_process(COMMAND chmod +x \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/save_image)")

# test stuff
if(BUILD_TESTING)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    find_package(launch_testing_ament_cmake)

    add_launch_test(test/test_launch_sim_works.py)
    add_test(
        NAME simulator_test_tf_tree
        COMMAND ${Python3_EXECUTABLE} -m pytest -q ${CMAKE_CURRENT_SOURCE_DIR}/test/test_tf_tree.py
    )
endif()

ament_package()
