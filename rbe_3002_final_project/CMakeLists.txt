cmake_minimum_required(VERSION 2.8.3)
project(rbe_3002_final_project)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  catkin
#  roscpp
  rospy
  nav_msgs
  geometry_msgs
  std_msgs
  message_generation
#  tf
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
	FILES
	CalculatePath.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS nav_msgs rospy std_msgs message_runtime
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

