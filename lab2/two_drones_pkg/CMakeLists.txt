cmake_minimum_required(VERSION 3.1)
project(two_drones_pkg)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf2_ros)

## Declare the current package and exported dependences
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

## Declare an executable to build
add_executable(frames_publisher_node src/frames_publisher_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(frames_publisher_node ${catkin_LIBRARIES})

## Declare an executable to build
add_executable(plots_publisher_node src/plots_publisher_node.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(plots_publisher_node ${catkin_LIBRARIES})
