cmake_minimum_required(VERSION 3.16)
project(two_drones_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(frames_publisher_node src/frames_publisher_node.cpp)
ament_target_dependencies(frames_publisher_node rclcpp std_msgs geometry_msgs tf2 tf2_ros)
target_include_directories(frames_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


add_executable(plots_publisher_node src/plots_publisher_node.cpp)
ament_target_dependencies(plots_publisher_node rclcpp std_msgs geometry_msgs visualization_msgs tf2 tf2_ros)
target_include_directories(plots_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS frames_publisher_node plots_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY mesh DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
