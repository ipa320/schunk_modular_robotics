cmake_minimum_required(VERSION 2.8.3)
project(schunk_sdh)

find_package(catkin REQUIRED COMPONENTS actionlib cob_srvs control_msgs diagnostic_msgs libntcan libpcan message_generation roscpp roslint sensor_msgs std_msgs std_srvs trajectory_msgs urdf)

find_package(Boost REQUIRED)

### Message Generation ###
add_message_files(
  DIRECTORY msg FILES
    TactileMatrix.msg
    TactileSensor.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

### BUILD ###
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

add_library(SDHLibrary SHARED IMPORTED)
add_dependencies(SDHLibrary copy_SDHLIB)
set_property(TARGET SDHLibrary PROPERTY IMPORTED_LOCATION ${PROJECT_SOURCE_DIR}/common/lib/libSDHLibrary-CPP.so)

add_executable(${PROJECT_NAME} ros/src/sdh.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} SDHLibrary ${catkin_LIBRARIES})

add_executable(sdh_only ros/src/sdh_only.cpp)
set_target_properties(sdh_only PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(sdh_only ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sdh_only SDHLibrary ${catkin_LIBRARIES})

add_executable(dsa_only ros/src/dsa_only.cpp)
set_target_properties(dsa_only PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX")
add_dependencies(dsa_only ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsa_only SDHLibrary ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME} sdh_only dsa_only
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### LINT ###
roslint_cpp(ros/src/sdh.cpp ros/src/dsa_only.cpp ros/src/sdh_only.cpp)
