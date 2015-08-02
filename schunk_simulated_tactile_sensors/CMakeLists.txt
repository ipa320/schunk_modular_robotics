cmake_minimum_required(VERSION 2.8.3)
project(schunk_simulated_tactile_sensors)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    CATKIN_DEPENDS rospy gazebo_msgs schunk_sdh
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS ros/src/simulated_tactile_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)
