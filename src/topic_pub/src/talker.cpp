#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include "mask_rcnn_ros/Result.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<std_msgs::String>("talker", 1000);
	ros::Rate loop_rate(1.0);
	int send_msg_num = 0;
	while(ros::ok())
	{
		std_msgs::String msg;
		std::stringstream ss;
		ss << "talker : send message " << send_msg_num << "...";
		msg.data = ss.str();
		ROS_INFO("%s", msg.data.c_str());
		pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
		++send_msg_num;
	}
	return 0;
}
