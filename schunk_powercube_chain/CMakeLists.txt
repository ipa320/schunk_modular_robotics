cmake_minimum_required(VERSION 2.8.3)
project(schunk_powercube_chain)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs urdf schunk_libm5api libntcan libpcan std_msgs std_srvs cob_srvs diagnostic_msgs control_msgs rostest)


if(CMAKE_COMPILER_IS_GNUCXX)
  add_definitions(-std=gnu++0x)
else()
  add_definitions(-std=c++0x)
endif()


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS sensor_msgs urdf schunk_libm5api libntcan libpcan std_msgs std_srvs cob_srvs diagnostic_msgs control_msgs
    DEPENDS # TODO
    INCLUDE_DIRS common/include
    LIBRARIES # TODO
)

###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

# add project executable
add_executable(${PROJECT_NAME} common/src/PowerCubeCtrl.cpp common/src/TimeStamp.cpp common/src/moveCommand.cpp ros/src/schunk_powercube_chain.cpp)

# link libraries
#target_link_libraries(${PROJECT_NAME} m5api)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD")


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
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

