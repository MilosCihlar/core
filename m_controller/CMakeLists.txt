cmake_minimum_required(VERSION 2.8.3)
project(m_controller)

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
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  m_controller 
    src/m_controller.cpp
    src/MController.cpp
    src/Point.cpp
    src/Position.cpp
    src/Quaternions.cpp
    src/Robot.cpp
    src/Wheel.cpp
)

target_link_libraries(
  m_controller 
  ${catkin_LIBRARIES}  
)
