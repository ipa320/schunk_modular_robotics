cmake_minimum_required(VERSION 2.8.3)
project(schunk_sdh)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS actionlib cob_srvs control_msgs diagnostic_msgs libntcan libpcan message_generation roscpp roslint sensor_msgs std_msgs std_srvs trajectory_msgs urdf)

find_package(Boost REQUIRED)

find_package(sdhlibrary_cpp REQUIRED)

### Message Generation ###
add_message_files(
  DIRECTORY msg FILES
    TactileMatrix.msg
    TactileSensor.msg
    TemperatureArray.msg
    PressureArray.msg
    PressureArrayList.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

### BUILD ###

include_directories(common/include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ros/src/sdh.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} SDHLibrary-CPP ${catkin_LIBRARIES})

add_executable(sdh_only ros/src/sdh_only.cpp)
set_target_properties(sdh_only PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX -DWITH_ESD_CAN")
add_dependencies(sdh_only ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sdh_only SDHLibrary-CPP ${catkin_LIBRARIES})

add_executable(dsa_only ros/src/dsa_only.cpp)
set_target_properties(dsa_only PROPERTIES COMPILE_FLAGS "-DOSNAME_LINUX")
add_dependencies(dsa_only ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dsa_only SDHLibrary-CPP ${catkin_LIBRARIES})

### INSTALL ###
install(TARGETS ${PROJECT_NAME} sdh_only dsa_only
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

### LINT ###
roslint_cpp(ros/src/sdh.cpp ros/src/dsa_only.cpp ros/src/sdh_only.cpp)
