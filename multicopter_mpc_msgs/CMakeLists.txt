cmake_minimum_required(VERSION 3.0.2)
project(multicopter_mpc_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    message_generation)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  FloatingBaseState.msg
  Mission.msg
  MotorsState.msg
  Thrust.msg
  Waypoint.msg
  WholeBodyState.msg
  WholeBodyTrajectory.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
