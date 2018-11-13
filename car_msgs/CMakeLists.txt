cmake_minimum_required(VERSION 2.8.3)
project(car_msgs)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	roscpp
	rospy
	std_msgs
)

add_message_files(
	FILES
	MotorsControl.msg
)

generate_messages(
  	DEPENDENCIES
  	std_msgs
)

catkin_package(
	CATKIN_DEPENDS message_runtime
)

include_directories(
	${catkin_INCLUDE_DIRS}
)