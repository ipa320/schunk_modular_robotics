cmake_minimum_required(VERSION 2.4.6)
project(schunk_description)

find_package(catkin REQUIRED COMPONENTS)

catkin_package()

install(DIRECTORY Media meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)