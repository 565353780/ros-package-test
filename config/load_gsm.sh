source ~/catkin_ws/devel/setup.bash
#{
#gnome-terminal -t "XXD_ros" -x bash -c "roscore;exec bash"
#}&

#sleep 1s
{
gnome-terminal -t "gsm" -x bash -c "roslaunch gsm_node scenenn_dataset.launch bag_file:=/home/chli/scenenn_231.bag;exec bash"
}&
