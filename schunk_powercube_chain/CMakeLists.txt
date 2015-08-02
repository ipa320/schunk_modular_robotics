cmake_minimum_required(VERSION 2.8.3)
project(schunk_powercube_chain)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs cob_srvs sensor_msgs diagnostic_msgs trajectory_msgs control_msgs urdf schunk_libm5api libntcan libpcan)

if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs std_srvs cob_srvs sensor_msgs diagnostic_msgs trajectory_msgs control_msgs urdf schunk_libm5api libntcan libpcan
  INCLUDE_DIRS common/include
)


### BUILD ###
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} common/src/PowerCubeCtrl.cpp common/src/TimeStamp.cpp common/src/moveCommand.cpp ros/src/schunk_powercube_chain.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD")


### INSTALL ###
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY common/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY orocos/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
