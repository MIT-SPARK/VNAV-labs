cmake_minimum_required(VERSION 3.1)
project(lab_8)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(
  catkin REQUIRED
  COMPONENTS actionlib
             darknet_ros
             gflags_catkin
             glog_catkin
             image_transport
             message_filters
             roscpp
             std_msgs
             geometry_msgs
             tf
             tf_conversions)
find_package(cv_bridge REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Version: ${OpenCV_VERSION_STRING} ${OpenCV_VERSION}")

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  CATKIN_DEPENDS
  roscpp
  std_msgs
  geometry_msgs
  gflags_catkin
  glog_catkin
  dbow2_catkin)

include_directories(include)

add_executable(deliverable_2 src/deliverable_2.cpp)
target_include_directories(
  deliverable_2 PRIVATE ${cv_bridge_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                        ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(
  deliverable_2
  PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam
  PRIVATE ${cv_bridge_LIBRARIES})

find_package(lab_5 REQUIRED)
add_library(lab5_wrapper src/tracker_wrapper.cpp)
target_include_directories(lab5_wrapper PRIVATE ${lab_5_INCLUDE_DIRS})
target_link_libraries(lab5_wrapper PRIVATE ${lab_5_LIBRARIES})

find_package(dbow2_catkin REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(glog_catkin REQUIRED)
find_package(roscpp REQUIRED)
add_executable(deliverable_3 src/deliverable_3.cpp src/euroc_dataset.cpp)
target_include_directories(
  deliverable_3 PUBLIC ${dbow2_catkin_INCLUDE_DIRS} ${glog_catkin_INCLUDE_DIRS}
                       ${roscpp_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS})
target_link_libraries(
  deliverable_3
  PUBLIC ${dbow2_catkin_LIBRARIES} ${geometry_msgs_LIBRARIES}
         ${roscpp_LIBRARIES} ${glog_catkin_LIBRARIES} ${OpenCV_LIBRARIES}
         lab5_wrapper)
