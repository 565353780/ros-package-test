source ~/catkin_ws/devel/setup.bash
#{
#gnome-terminal -t "XXD_ros" -x bash -c "roscore;exec bash"
#}&

#sleep 1s
{
gnome-terminal -t "gazebo" -x bash -c "roslaunch gsm_topic_sync single_robot.launch;exec bash"
}&

sleep 4s
{
gnome-terminal -t "topic" -x bash -c "rosrun gsm_topic_sync gsm_topic_sync;exec bash"
}&
 
sleep 1s
{
gnome-terminal -t "rviz" -x bash -c "rosrun rviz rviz;exec bash"
}&

sleep 1s
{
gnome-terminal -t "gsm" -x bash -c "roslaunch gsm_node vpp_pipeline.launch;exec bash"
}
