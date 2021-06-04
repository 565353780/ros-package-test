source ~/catkin_ws/devel/setup.bash
#{
#gnome-terminal -t "XXD_ros" -x bash -c "roscore;exec bash"
#}&

#sleep 1s
{
gnome-terminal -t "trans_camera_depth_image" -x bash -c "rosrun trans_camera_depth_image trans_camera_depth_image;exec bash"
}&
 
#sleep 4s
{
gnome-terminal -t "trans_camera_depth_info" -x bash -c "rosrun trans_camera_depth_info trans_camera_depth_info;exec bash"
}&
 
#sleep 1s
{
gnome-terminal -t "trans_camera_rgb_image" -x bash -c "rosrun trans_camera_rgb_image trans_camera_rgb_image;exec bash"
}&
 
#sleep 1s
{
gnome-terminal -t "trans_camera_rgb_info" -x bash -c "rosrun trans_camera_rgb_info trans_camera_rgb_info;exec bash"
}&

#sleep 1s
{
gnome-terminal -t "trans_camera_tf" -x bash -c "rosrun trans_camera_tf trans_camera_tf;exec bash"
}&
