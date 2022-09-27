cmake_minimum_required(VERSION 2.8.3)
project(planner_pkg)

include(CheckCXXCompilerFlag)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wl,--no-as-needed")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  mav_msgs
  eigen_conversions
	tf
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  mav_msgs 
  tf
  eigen_conversions
)

find_package(Eigen3)

set(HEADER_FILES include/csv.h)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(traj_vertices_publisher src/traj_vertices_publisher.cpp ${HEADER_FILES})
add_dependencies(traj_vertices_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(traj_vertices_publisher ${catkin_LIBRARIES})

add_executable(simple_traj_planner src/simple_traj_planner.cpp)
add_dependencies(simple_traj_planner ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_traj_planner ${catkin_LIBRARIES})
