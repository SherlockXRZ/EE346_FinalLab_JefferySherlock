cmake_minimum_required(VERSION 2.8.3)
project(lane_following)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp rospy std_msgs genmsg geometry_msgs)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
##add_message_files(FILES pos.msg)

## Generate added messages and services
##generate_messages(DEPENDENCIES std_msgs)

catkin_package()


## Executables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/lane_following.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# add_executable(lane_following src/lane_following.cpp)
# target_link_libraries(lane_following ${catkin_LIBRARIES} ${OpenCV_LIBS})
# add_dependencies(lane_following lane_following_gencpp)
