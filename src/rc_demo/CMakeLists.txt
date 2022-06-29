cmake_minimum_required(VERSION 2.8.3)
project(rc_demo)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV 3.2 COMPONENTS core highgui imgproc video REQUIRED)

#add_dependencies(mavros_control_node mavros_control_messages_cpp)
#add_dependencies(mavros_comm_node mavros_control_messages_cpp)
