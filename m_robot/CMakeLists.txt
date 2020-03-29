cmake_minimum_required(VERSION 2.8.3)
project(m_robot)

##Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        )

##Declare a catkin package
catkin_package()

##Build
include_directories(include
        ${catkin_INCLUDE_DIRS})

add_executable(m_robot
        src/robot.cpp
        )

target_link_libraries(m_robot
        ${catkin_LIBRARIES})