cmake_minimum_required(VERSION 2.8.3)
project(slam_quality)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rosbag
        std_msgs
        nav_msgs
        )

##Declare a catkin package
catkin_package()

##Build
include_directories(include
        ${catkin_INCLUDE_DIRS}
        )

add_executable(
        slam_quality
        src/slam_quality.cpp
        src/Point.cpp
        src/Map.cpp
)

target_link_libraries(
        slam_quality
        ${catkin_LIBRARIES}
)
