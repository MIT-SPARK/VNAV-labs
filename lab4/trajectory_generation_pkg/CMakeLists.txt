cmake_minimum_required(VERSION 3.16)
project(trajectory_generation)

add_definitions(-std=c++17)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(mav_planning_msgs REQUIRED)
find_package(mav_trajectory_generation REQUIRED)
find_package(mav_visualization REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(NLopt REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)

#find_package(catkin REQUIRED COMPONENTS
#  catkin_simple
#  cmake_modules
#  eigen_catkin
#  eigen_conversions
#  eigen_checks
#  glog_catkin
#  mav_msgs
#  geometry_msgs
#  nav_msgs
#  trajectory_msgs
#  mav_planning_msgs
#  mav_trajectory_generation
#  mav_visualization
#  nlopt
#  roslib
#  yaml_cpp_catkin
#)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/trajectory_generation_node.cpp)
message("mtg include: " ${mav_trajectory_generation_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_node geometry_msgs nav_msgs rclcpp trajectory_msgs mav_trajectory_generation Eigen3 tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME}_node glog::glog)
target_include_directories(${PROJECT_NAME}_node PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)


## Add cmake target dependencies of the executable
## same as for the library above
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(${PROJECT_NAME}_node
#        ${catkin_LIBRARIES}
#)


add_executable(spiral_pose_publisher_node src/spiral_pose_publisher.cpp)
ament_target_dependencies(spiral_pose_publisher_node geometry_msgs rclcpp tf2_geometry_msgs)
#add_dependencies(spiral_pose_publisher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(spiral_pose_publisher_node
#        ${catkin_LIBRARIES}
#        )

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

ament_package()
