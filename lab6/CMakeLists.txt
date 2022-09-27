cmake_minimum_required(VERSION 2.8.3)
project(lab_6)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  opencv3_catkin
  glog_catkin
  image_transport
  cv_bridge
  lab_5
  opengv
  eigen_catkin
  tf_conversions
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS roscpp rospy opencv3_catkin glog_catkin image_transport cv_bridge lab_5 opengv eigen_catkin
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(pose_estimation src/pose_estimation.cpp)
target_link_libraries(pose_estimation ${catkin_LIBRARIES})

