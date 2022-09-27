cmake_minimum_required(VERSION 2.8.3)
project(tesse_ros_bridge)

find_package(catkin_simple REQUIRED)

catkin_python_setup()
catkin_simple()

cs_install()
# cs_install_scripts(scripts/tesse_ros_bridge.py)

cs_export()
