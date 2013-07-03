# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(schunk_sdh)


find_package(catkin REQUIRED COMPONENTS roscpp urdf actionlib libntcan libpcan message_generation sensor_msgs trajectory_msgs cob_srvs control_msgs diagnostic_msgs brics_actuator)

#######################################
## Declare ROS messages and services ##
#######################################

add_message_files(
  DIRECTORY msg
  FILES
    TactileMatrix.msg 
    TactileSensor.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES trajectory_msgs sensor_msgs std_msgs control_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    DEPENDS roscpp urdf sensor_msgs trajectory_msgs actionlib cob_srvs control_msgs libntcan libpcan diagnostic_msgs brics_actuator
    CATKIN_DEPENDS message_runtime
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

###########
## Build ##
###########
include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ros/src/sdh.cpp )
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
#add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp ros/src/sdhWrapper.cpp)
#add_executable(${PROJECT_NAME}_demo ros/src/${PROJECT_NAME}_demo.cpp ros/src/dsaboost.cpp)
#add_executable(${PROJECT_NAME}_driver ros/src/${PROJECT_NAME}_driver.cpp ros/src/dsaboost.cpp)

#target_link_libraries(${PROJECT_NAME} )
#target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan)
#message(STATUS "Linking ${catkin_LIBRARIES}")

target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.so ${catkin_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan pcan)
#target_link_libraries(${PROJECT_NAME}_driver ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan pcan)

# SDH only
add_executable(sdh_only ros/src/sdh_only.cpp )
target_link_libraries(sdh_only ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.so ${catkin_LIBRARIES})
set_target_properties(sdh_only PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(sdh_only ${PROJECT_NAME}_gencpp)

# DSA only
add_executable(dsa_only ros/src/dsa_only.cpp )
target_link_libraries(dsa_only ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.so ${catkin_LIBRARIES})
set_target_properties(dsa_only PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX")
add_dependencies(dsa_only ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

