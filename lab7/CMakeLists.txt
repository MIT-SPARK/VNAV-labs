cmake_minimum_required(VERSION 2.8.3)
project(lab_7)

add_compile_options(-std=c++14)

find_package(catkin_simple REQUIRED)
find_package(GTSAM REQUIRED)

# Boost - same requirement as gtsam
find_package(Boost 1.50 REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

catkin_simple()

cs_add_executable(deliverable_1 src/deliverable_1.cpp)
target_link_libraries(deliverable_1 ${Boost_LIBRARIES} ${catkin_LIBRARIES} gtsam)

cs_add_executable(deliverable_2_3 src/deliverable_2_3.cpp)
target_link_libraries(deliverable_2_3 ${Boost_LIBRARIES} ${catkin_LIBRARIES} gtsam)

cs_add_executable(deliverable_4 src/deliverable_4.cpp)
target_link_libraries(deliverable_4 ${Boost_LIBRARIES} ${catkin_LIBRARIES} gtsam)

cs_add_executable(deliverable_5 src/deliverable_5.cpp)
target_link_libraries(deliverable_5 ${Boost_LIBRARIES} ${catkin_LIBRARIES} gtsam)
