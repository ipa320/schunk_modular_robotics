cmake_minimum_required(VERSION 2.8.3)
project(schunk_powercube_chain)

find_package(catkin REQUIRED COMPONENTS cob_srvs control_msgs diagnostic_msgs libntcan libpcan roscpp schunk_libm5api sensor_msgs std_msgs std_srvs trajectory_msgs urdf)

catkin_package()

### BUILD ###
include_directories(common/include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} common/src/PowerCubeCtrl.cpp common/src/TimeStamp.cpp common/src/moveCommand.cpp ros/src/schunk_powercube_chain.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD")
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

