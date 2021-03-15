cmake_minimum_required(VERSION 3.0.2)
project(multicopter_mpc_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    dynamic_reconfigure
)

catkin_package(
CATKIN_DEPENDS std_msgs
)

catkin_install_python(PROGRAMS 
scripts/disturbance_trigger.py 

DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)