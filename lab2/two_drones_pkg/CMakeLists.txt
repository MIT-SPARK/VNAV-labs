cmake_minimum_required(VERSION 2.8.3)
project(two_drones_pkg)
set(CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf)

catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(frames_publisher_node src/frames_publisher_node.cpp)
add_executable(plots_publisher_node src/plots_publisher_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(frames_publisher_node ${catkin_LIBRARIES})
target_link_libraries(plots_publisher_node ${catkin_LIBRARIES})
