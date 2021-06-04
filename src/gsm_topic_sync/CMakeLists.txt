cmake_minimum_required(VERSION 3.0.2)
project(gsm_topic_sync)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED  COMPONENTS roscpp rospy std_msgs genmsg message_filters tf)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
 
add_executable(gsm_topic_sync src/gsm_topic_sync.cpp)
target_link_libraries(gsm_topic_sync ${catkin_LIBRARIES})

