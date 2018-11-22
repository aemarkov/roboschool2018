cmake_minimum_required(VERSION 2.8.3)
project(car_hackathon)

set (CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  car_msgs
  cv_bridge
)
find_package(OpenCV)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_mover_cpp src/line_mover.cpp)
target_link_libraries(line_mover_cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)