cmake_minimum_required(VERSION 3.5)
project(planner_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mav_msgs REQUIRED)
#find_package(eigen_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Eigen3)

#set(HEADER_FILES include/csv.h)
include_directories(include)

add_executable(traj_vertices_publisher src/traj_vertices_publisher.cpp ${HEADER_FILES})
ament_target_dependencies(traj_vertices_publisher geometry_msgs trajectory_msgs rclcpp)

add_executable(simple_traj_planner src/simple_traj_planner.cpp)
ament_target_dependencies(simple_traj_planner geometry_msgs mav_msgs trajectory_msgs tf2_ros rclcpp std_msgs)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS traj_vertices_publisher simple_traj_planner
  DESTINATION lib/${PROJECT_NAME})

ament_package()
