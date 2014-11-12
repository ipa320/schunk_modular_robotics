# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(schunk_simulated_tactile_sensors)

find_package(catkin REQUIRED COMPONENTS rospy schunk_sdh geometry_msgs rostest)

catkin_package(
    CATKIN_DEPENDS rospy schunk_sdh geometry_msgs
    DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)


# rostest
#rosbuild_add_roslaunch_check(ros/launch)## Generate added messages and services with any dependencies listed here

## Mark cpp header files for installation
install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(PROGRAMS ros/src/simulated_tactile_sensors.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/ros/src
)
