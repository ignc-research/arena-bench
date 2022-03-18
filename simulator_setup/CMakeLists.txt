cmake_minimum_required(VERSION 3.0.2)
project(simulator_setup)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  geometry_msgs
  roscpp
  rospy
  gazebo_msgs
  urdf
  nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED )
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################



################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_ros roscpp  geometry_msgs  std_msgs nav_msgs tf message_runtime
)


###########
## Build ##
###########


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)


#############
## Install ##
#############


install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
