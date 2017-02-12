cmake_minimum_required(VERSION 2.8.3)
project(mw_rtkrcv)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  GpsLocation.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mw_rtkrcv
  CATKIN_DEPENDS message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(mw_rtkrcv
#   src/${PROJECT_NAME}/mw_rtkrcv.cpp
#)

add_executable(rtkrcv src/binex.c src/crescent.c src/ephemeris.c src/geoid.c src/gw10.c src/ionex.c src/javad.c src/lambda.c src/novatel.c src/nvs.c src/options.c src/pntpos.c src/ppp.c src/ppp_ar.c src/ppp_corr.c src/preceph.c src/qzslex.c src/rcvraw.c src/rinex.c src/rt17.c src/rtcm.c src/rtcm2.c src/rtcm3.c src/rtcm3e.c src/rtkcmn.c src/rtkpos.c src/rtkrcv.c src/rtkrcv_publish.cpp src/rtksvr.c src/sbas.c src/septentrio.c src/skytraq.c src/solution.c src/ss2.c src/stream.c src/tides.c src/ublox.c src/vt.c)
add_dependencies(rtkrcv mw_rtkrcv_generate_messages_cpp)

set_target_properties(rtkrcv PROPERTIES COMPILE_FLAGS "-mfpu=neon -ffast-math -DENAGLO -DENAQZS -DENACMP -DNFREQ=3 -Wall -O3 -ansi -pedantic -Wno-unused-but-set-variable -Isrc -I. -DTRACE ")
target_link_libraries(rtkrcv ${catkin_LIBRARIES} -lm -lrt -lpthread)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS mw_video mw_video_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

install(DIRECTORY nodes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mw_video.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
