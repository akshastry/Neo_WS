cmake_minimum_required(VERSION 2.8.3)
project(realsense)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED)

catkin_package()


add_executable(realsense_pose src/pose.cpp)
target_link_libraries(realsense_pose librealsense2.so)


