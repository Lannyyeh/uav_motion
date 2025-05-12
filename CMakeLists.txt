cmake_minimum_required(VERSION 2.8.3)
project(uav_motion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin_simple REQUIRED
        mavros_msgs
        std_msgs
        message_generation
)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_service_files(
        FILES
        release.srv
        generatePath.srv
)

catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_MACOSX_RPATH 0)
add_definitions(-std=c++11)

cs_add_executable(trajectory_generator
        src/trajectory_generator.cpp
)

cs_add_executable(trajectory_sampler
        src/trajectory_sampler.cpp
)

cs_add_executable(geometric_controller
        src/geometric_controller.cpp
)

catkin_install_python(PROGRAMS
  scripts/waypoint_generator_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/waypoint_generator2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/simple_waypoint_generator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

##########
# EXPORT #
##########
cs_install()
cs_export()
