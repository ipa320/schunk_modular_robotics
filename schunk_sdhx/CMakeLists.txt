cmake_minimum_required(VERSION 2.8.3)
project(schunk_sdhx)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  roslib
  rospy
  sensor_msgs
)


#######################################
## Declare ROS messages and services ##
#######################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS rospy control_msgs roslib sensor_msgs# TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/sdhx_node.py src/recover_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

