cmake_minimum_required(VERSION 3.8)
project(controller_pkg)

# ----------------------------------------------------------
# Don't edit
# ----------------------------------------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ----------------------------------------------------------

find_package(ament_cmake REQUIRED)
#COMPONENTS
#  roscpp 
#  rospy 
#  std_msgs 
#  geometry_msgs 
#  mav_msgs
#  tf
#  tf2
#)

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

#include_directories(include)
add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  rclcpp
  Eigen3
  mav_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

add_executable(traj_publisher src/traj_publisher.cpp)
ament_target_dependencies(traj_publisher
  rclcpp
  Eigen3
  mav_msgs
  nav_msgs
  trajectory_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
)

install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  traj_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  #rviz
  #worlds
  #urdf
  #meshes
  DESTINATION share/${PROJECT_NAME}
)

# ----------------------------------------------------------
# Don't edit
# ----------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
# ----------------------------------------------------------