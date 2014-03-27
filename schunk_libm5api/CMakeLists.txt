# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(schunk_libm5api)


find_package(catkin REQUIRED COMPONENTS libpcan libntcan roslib)


###################################
## catkin specific configuration ##
###################################
# TODO: fill in what other packages will need to use this package
## DEPENDS: system dependencies of this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## INCLUDE_DIRS: 
## LIBRARIES: libraries you create in this project that dependent projects also need
catkin_package(
    DEPENDS
    CATKIN_DEPENDS libpcan libntcan roslib
    INCLUDE_DIRS include
    LIBRARIES m5api
)

###########
## Build ##
###########

# add include search paths
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#common commands for building c++ executables and libraries
add_library(m5api src/Device/Device.cpp  src/Device/ProtocolDevice.cpp src/Device/ESDDevice.cpp src/Device/PCanDevice.cpp src/Device/ProtocolMessage.cpp  src/Device/RS232Device.cpp  src/Util/Random.cpp  src/Util/Message.cpp src/Util/StopWatch.cpp src/Util/IOFunctions.cpp  src/Util/Thread.cpp  src/M5apiw32/m5apiw32.cpp)
set_target_properties(m5api PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_PCAN -DUSE_ESD")
target_link_libraries(m5api ${catkin_LIBRARIES})



#############
## Install ##
#############
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY src/ComDef/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/ComDef
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY src/Device/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Device
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY src/Util/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/Util
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark executables and/or libraries for installation
install(TARGETS m5api
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY include
#  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#)
