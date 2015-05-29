cmake_minimum_required(VERSION 2.8.3)
project(schunk_sdh)

find_package(catkin REQUIRED COMPONENTS roscpp urdf actionlib libntcan libpcan message_generation std_msgs std_srvs sensor_msgs trajectory_msgs cob_srvs control_msgs diagnostic_msgs)

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
    CATKIN_DEPENDS roscpp message_runtime urdf std_msgs sensor_msgs trajectory_msgs actionlib cob_srvs std_srvs control_msgs libntcan libpcan diagnostic_msgs
)

###########
## Build ##
###########
find_program(CMAKE_DPKG dpkg /bin /usr/bin /usr/local/bin )
if(CMAKE_DPKG)
  exec_program(dpkg ARGS --print-architecture OUTPUT_VARIABLE CMAKE_DPKG_ARCH)
  if(CMAKE_DPKG_ARCH MATCHES "amd64")
    message(STATUS "++++++++++++++++++++++++++ DETECTED 64 bit +++++++++++++++++++++++++++++++++++")
    set(MYARCH "x86_64")
  else()
    message(STATUS "++++++++++++++++++++++++++ DETECTED 32 bit +++++++++++++++++++++++++++++++++++")
    set(MYARCH "i686")
  endif()
endif()



add_custom_target(
    copy_SDHLIB ALL 
    COMMAND /bin/ln -sf ${PROJECT_SOURCE_DIR}/common/lib/${MYARCH}/libSDHLibrary-CPP.so ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.so
    DEPENDS ${PROJECT_SOURCE_DIR}/common/lib/${MYARCH}/libSDHLibrary-CPP.so
)

include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_library(SDHLibrary SHARED IMPORTED )
add_dependencies(SDHLibrary copy_SDHLIB)
set_property(TARGET SDHLibrary PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.so)

add_executable(${PROJECT_NAME} ros/src/sdh.cpp )
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
#add_executable(${PROJECT_NAME} ros/src/${PROJECT_NAME}.cpp ros/src/sdhWrapper.cpp)
#add_executable(${PROJECT_NAME}_demo ros/src/${PROJECT_NAME}_demo.cpp ros/src/dsaboost.cpp)
#add_executable(${PROJECT_NAME}_driver ros/src/${PROJECT_NAME}_driver.cpp ros/src/dsaboost.cpp)

#target_link_libraries(${PROJECT_NAME} )
#target_link_libraries(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan)
#message(STATUS "Linking ${catkin_LIBRARIES}")

target_link_libraries(${PROJECT_NAME} SDHLibrary ${catkin_LIBRARIES})
#target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan pcan)
#target_link_libraries(${PROJECT_NAME}_driver ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.a ntcan pcan)

# SDH only
add_executable(sdh_only ros/src/sdh_only.cpp )
target_link_libraries(sdh_only SDHLibrary ${catkin_LIBRARIES})
set_target_properties(sdh_only PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(sdh_only ${PROJECT_NAME}_gencpp)

# DSA only
add_executable(dsa_only ros/src/dsa_only.cpp )
target_link_libraries(dsa_only SDHLibrary ${catkin_LIBRARIES})
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

install(TARGETS sdh_only
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS dsa_only
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(TARGETS SDHLibrary-CPP
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark cpp header files for installation
install(DIRECTORY common/include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

