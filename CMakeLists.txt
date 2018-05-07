cmake_minimum_required(VERSION 3.9)
project(lookup)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        geometry_msgs
        std_msgs
        )

include_directories(${catkin_INCLUDE_DIRS})
add_executable(lookup main.cpp tool.h)
target_link_libraries(lookup ${catkin_LIBRARIES})