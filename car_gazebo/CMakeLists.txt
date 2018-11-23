cmake_minimum_required(VERSION 2.8.3)
project(car_gazebo)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  car_msgs
)
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rospy gazebo_ros car_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(car_plugin SHARED src/CarPlugin/CarPlugin.cpp)
target_link_libraries(car_plugin ${GAZEBO_LIBRARIES})
add_dependencies(car_plugin ${catkin_EXPORTED_TARGETS})

add_library(light_plugin SHARED src/LightPlugin/LightPlugin.cpp)
target_link_libraries(light_plugin ${GAZEBO_LIBRARIES})
