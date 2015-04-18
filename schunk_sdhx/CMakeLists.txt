cmake_minimum_required(VERSION 2.8.3)
project(schunk_sdhx)

find_package(catkin REQUIRED COMPONENTS
  roslib
  std_srvs
  sensor_msgs
  control_msgs
)

catkin_package(
    CATKIN_DEPENDS rospy roslib std_srvs sensor_msgs control_msgs
)


## Mark executables and/or libraries for installation
install(TARGETS 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/sdhx_node.py src/recover_gripper.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src
)

