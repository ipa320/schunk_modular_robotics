cmake_minimum_required(VERSION 2.8.3)
project(schunk_powercube_chain)

find_package(catkin REQUIRED COMPONENTS cob_srvs control_msgs diagnostic_msgs libntcan libpcan roscpp schunk_libm5api sensor_msgs std_msgs std_srvs trajectory_msgs urdf)

find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS schunk_libm5api
  DEPENDS Boost
  INCLUDE_DIRS common/include
)

### BUILD ###
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} common/src/PowerCubeCtrl.cpp common/src/TimeStamp.cpp common/src/moveCommand.cpp ros/src/schunk_powercube_chain.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD")
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
