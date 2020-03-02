cmake_minimum_required(VERSION 2.8.3)
project(m_exploration)

##Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

##Declare a catkin package
catkin_package()

##Build
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  m_exploration 
    src/m_exploration.cpp
    src/Exploration.cpp
    src/Map.cpp
    src/Point.cpp
    src/Tree.cpp
)

target_link_libraries(
  m_exploration
  ${catkin_LIBRARIES}  
)
