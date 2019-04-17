
cmake_minimum_required(VERSION 2.8.3)
project(move_basic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  geometry_msgs
  std_msgs
  actionlib
  actionlib_msgs
  sensor_msgs
  visualization_msgs
  move_base_msgs
  nav_core
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  FollowMode.msg
)
generate_messages(
  DEPENDENCIES 
  std_msgs 
)

catkin_package(INCLUDE_DIRS DEPENDS)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS} include)

add_executable(move_basic src/obstacle_detector.cpp src/move_basic.cpp)
add_dependencies(move_basic ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(move_basic ${catkin_LIBRARIES})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS move_basic
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY launch/
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
#)

###########
## Tests ##
###########

if(CATKIN_ENABLE_TESTING)
        find_package(rostest REQUIRED)

        add_rostest_gtest(obstacle_test test/obstacles.test
                src/obstacle_detector.cpp
                test/test_obstacles.cpp)
        target_link_libraries(obstacle_test ${catkin_LIBRARIES})
endif()

