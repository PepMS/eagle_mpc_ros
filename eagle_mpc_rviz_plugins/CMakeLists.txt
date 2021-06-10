cmake_minimum_required(VERSION 3.0.2)
project(eagle_mpc_rviz_plugins)

SET(PROJECT_NAME eagle_mpc_rviz_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
MESSAGE(STATUS "${PROJECT_DESCRIPTION}, version ${PROJECT_VERSION}")

find_package(catkin REQUIRED COMPONENTS
roscpp
rviz
eagle_mpc_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)

set(QT_LIBRARIES Qt5::Widgets)

qt5_wrap_cpp(MOC_FILES
    include/eagle_mpc_rviz_plugins/ArrowVisual.h
    include/eagle_mpc_rviz_plugins/PinocchioLinkUpdater.h
    include/eagle_mpc_rviz_plugins/PointVisual.h
    include/eagle_mpc_rviz_plugins/WholeBodyStateDisplay.h
    include/eagle_mpc_rviz_plugins/WholeBodyTrajectoryDisplay.h
    OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
   
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
)

INCLUDE_DIRECTORIES(
  include 
  ${catkin_INCLUDE_DIRS} 
  ${Boost_INCLUDE_DIRS} 
  ${pinocchio_INCLUDE_DIRS}
  )

LINK_DIRECTORIES(${catkin_LIBRARY_DIRS}
                 ${Boost_LIBRARY_DIRS})

SET(SOURCE_FILES 
src/ArrowVisual.cpp
src/PointVisual.cpp 
src/WholeBodyStateDisplay.cpp 
src/PinocchioLinkUpdater.cpp
src/WholeBodyTrajectoryDisplay.cpp
${MOC_FILES}
)

ADD_LIBRARY(${PROJECT_NAME}  ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(${PROJECT_NAME}  ${QT_LIBRARIES}
                                       ${Boost_LIBRARIES}
                                       ${catkin_LIBRARIES}
                                       ${pinocchio_LIBRARIES})
                                       
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-ignored-attributes)  # Silence Eigen::Tensor warnings
# ADD_DEPENDENCIES(${PROJECT_NAME}  ${state_msgs_EXPORTED_TARGETS})

INSTALL(FILES ${CMAKE_SOURCE_DIR}/plugin_description.xml DESTINATION share/${PROJECT_NAME})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)