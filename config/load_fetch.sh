source ~/catkin_ws/devel/setup.bash
#{
#gnome-terminal -t "XXD_ros" -x bash -c "roscore;exec bash"
#}&

#sleep 1s
{
gnome-terminal -t "fetch_playground" -x bash -c "roslaunch fetch_gazebo playground.launch;exec bash"
}&
 
sleep 4s
{
gnome-terminal -t "rviz" -x bash -c "rosrun rviz rviz;exec bash"
}&
 
#sleep 1s
{
gnome-terminal -t "moveit" -x bash -c "roslaunch fetch_moveit_config move_group.launch;exec bash"
}&
 
#sleep 1s
{
gnome-terminal -t "controller" -x bash -c "rosrun teleop_twist_keyboard teleop_twist_keyboard.py;exec bash"
}&

sleep 10s
{
gnome-terminal -t "python-controller" -x bash -c "python fetch_controller.py;exec bash"
}&
