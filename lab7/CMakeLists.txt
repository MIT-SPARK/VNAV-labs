cmake_minimum_required(VERSION 3.16)
project(lab_7)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(GTSAM REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(glog REQUIRED)

add_executable(deliverable_1 src/deliverable_1.cpp)
target_link_libraries(deliverable_1 gtsam)
target_include_directories(deliverable_1 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(deliverable_2_3 src/deliverable_2_3.cpp)
ament_target_dependencies(deliverable_2_3 geometry_msgs visualization_msgs std_msgs rclcpp)
target_link_libraries(deliverable_2_3 gtsam glog::glog)
target_include_directories(deliverable_2_3 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(deliverable_4 src/deliverable_4.cpp)
ament_target_dependencies(deliverable_4 geometry_msgs visualization_msgs std_msgs rclcpp)
target_link_libraries(deliverable_4 gtsam glog::glog)
target_include_directories(deliverable_4 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(deliverable_5 src/deliverable_5.cpp)
target_link_libraries(deliverable_5 gtsam)
target_include_directories(deliverable_5 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS deliverable_1 deliverable_2_3 deliverable_4 deliverable_5 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})


ament_package()
