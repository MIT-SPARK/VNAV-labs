cmake_minimum_required(VERSION 2.8.3)
project(lab_5)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  opencv3_catkin
  glog_catkin
  image_transport
  cv_bridge
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES feature_trackers
 CATKIN_DEPENDS roscpp rospy opencv3_catkin glog_catkin image_transport cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#  DELIVERABLE 6 (continued) | Comparing Feature Matching on Real Data
# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
#
# Don't forget to modify the CMakeLists.txt to add your ORB and FAST+BRIEF
# feature trackers!
#
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
add_library(feature_trackers src/feature_tracker.cpp
                             src/sift_feature_tracker.cpp
                             src/surf_feature_tracker.cpp
                             src/orb_feature_tracker.cpp
                             src/fast_feature_tracker.cpp
                             src/lk_feature_tracker.cpp)

target_link_libraries(feature_trackers ${catkin_LIBRARIES})

add_executable(track_features src/track_features.cpp)
target_link_libraries(track_features feature_trackers ${catkin_LIBRARIES})

add_executable(self_flow src/self_flow.cpp)
target_link_libraries(self_flow feature_trackers ${catkin_LIBRARIES})
