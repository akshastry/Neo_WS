cmake_minimum_required(VERSION 2.8.3)
project(odom_pkg)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(serialcom src/Serial_port_read1.cpp)
target_link_libraries(serialcom ${catkin_LIBRARIES})
add_dependencies(serialcom odom_pkg_cpp)


