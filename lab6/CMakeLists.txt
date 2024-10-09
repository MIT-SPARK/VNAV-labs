cmake_minimum_required(VERSION 3.16)
project(lab_6)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(glog REQUIRED)
find_package(lab_5 REQUIRED)
find_package(opengv REQUIRED)
find_package (OpenCV 4.0.0 REQUIRED)

add_executable(pose_estimation src/pose_estimation.cpp)
ament_target_dependencies(pose_estimation geometry_msgs cv_bridge image_transport nav_msgs tf2 tf2_ros tf2_eigen lab_5 opengv tf2_geometry_msgs)
target_link_libraries(pose_estimation glog::glog opengv)
target_include_directories(pose_estimation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS pose_estimation DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY bags DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
