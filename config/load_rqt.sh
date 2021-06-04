source ~/catkin_ws/devel/setup.bash
#{
#gnome-terminal -t "XXD_ros" -x bash -c "roscore;exec bash"
#}&

#sleep 1s
{
gnome-terminal -t "rqt_graph" -x bash -c "rosrun rqt_graph rqt_graph;exec bash"
}&
