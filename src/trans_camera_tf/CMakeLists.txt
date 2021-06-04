cmake_minimum_required(VERSION 3.0.2)
project(trans_camera_tf)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED  COMPONENTS roscpp rospy std_msgs genmsg tf)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
 
add_executable(trans_camera_tf src/trans_camera_tf.cpp)
target_link_libraries(trans_camera_tf ${catkin_LIBRARIES})
