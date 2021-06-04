cmake_minimum_required(VERSION 3.0.2)
project(trans_camera_depth_info)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED  COMPONENTS roscpp rospy std_msgs genmsg)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
 
add_executable(trans_camera_depth_info src/trans_camera_depth_info.cpp)
target_link_libraries(trans_camera_depth_info ${catkin_LIBRARIES})
