cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_definitions(-std=c++11)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(step_plugin SHARED step_plugin.cc)
target_link_libraries(step_plugin ${GAZEBO_libraries} ${roscpp_LIBRARIES})