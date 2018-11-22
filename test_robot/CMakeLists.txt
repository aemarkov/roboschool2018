cmake_minimum_required(VERSION 2.8.3)
project(test_robot)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  car_msgs
)
find_package(gazebo REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(test_robot_plugin SHARED src/TestRobotPlugin/TestRobotPlugin.cpp)
target_link_libraries(test_robot_plugin ${GAZEBO_LIBRARIES})