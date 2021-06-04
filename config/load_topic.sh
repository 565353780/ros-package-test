source ~/catkin_ws/devel/setup.bash
#{
#gnome-terminal -t "XXD_ros" -x bash -c "roscore;exec bash"
#}&

#sleep 1s
{
gnome-terminal -t "topic_list" -x bash -c "rostopic list;exec bash"
}&
 
#sleep 4s
{
gnome-terminal -t "topic_type" -x bash -c "rostopic list;exec bash"
}&
 
#sleep 1s
{
gnome-terminal -t "topic_echo" -x bash -c "rostopic list;exec bash"
}&
