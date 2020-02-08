cmake_minimum_required(VERSION 2.8.3)
project(keyboard_driver)

##Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

##Declare a catkin package
catkin_package()

##Build
include_directories(include
${catkin_INCLUDE_DIRS})

add_executable(keyboard_driver
  src/keys_publisher.cpp
)

target_link_libraries(keyboard_driver 
${catkin_LIBRARIES})