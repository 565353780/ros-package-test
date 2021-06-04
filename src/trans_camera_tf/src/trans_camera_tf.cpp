#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

class TopicTranslator
{
public:
    TopicTranslator()
    {
	pThis=this;
    }

    ~TopicTranslator()
    {
	delete n_;
    }

    void init()
    {
        n_ = new ros::NodeHandle();
    }

    void setSubTopic(std::string topic)
    {
        sub_ = n_->subscribe(topic, 1000, this->SubscriberCallback);
    }

    void startTrans()
    {
        ros::spin();
    }

private:
    void static SubscriberCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double position_x = msg->pose.pose.position.x;
        double position_y = msg->pose.pose.position.y;
        double position_z = msg->pose.pose.position.z;

        double orientation_x = msg->pose.pose.orientation.x;
        double orientation_y = msg->pose.pose.orientation.y;
        double orientation_z = msg->pose.pose.orientation.z;
        double orientation_w = msg->pose.pose.orientation.w;

        tf::Transform transform;
        transform.setOrigin(tf::Vector3(position_x, position_y, position_z));
        tf::Quaternion q;
        q.setRotation(tf::Vector3(orientation_x, orientation_y, orientation_z), orientation_w);
        transform.setRotation(q);

	    pThis->pub_.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "arm_camera_rgb_optical_frame"));
        ROS_INFO("I heard: [s]");
    }
    
private:
    static TopicTranslator* pThis;

    ros::NodeHandle* n_;

    ros::Subscriber sub_;
    tf::TransformBroadcaster pub_;
};

TopicTranslator* TopicTranslator::pThis = nullptr;

///arm_camera/depth_registered/camera_info
///arm_camera/depth_registered/image_raw
///arm_camera/rgb/camera_info
///arm_camera/rgb/image_raw
///arm_camera_pose_ground_truth

///camera/depth/camera_info
///camera/depth/image_raw
///camera/rgb/camera_info
///camera/rgb/image_raw
///tf

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trans_camera_tf");

    TopicTranslator topic_translator;

    topic_translator.init();

    topic_translator.setSubTopic("arm_camera_pose_ground_truth");

    topic_translator.startTrans();

//   ros::init(argc, argv, "topic_translator");

//   ros::NodeHandle n;

//   ros::Subscriber sub = n.subscribe("mask_rcnn/result", 1000, SubscriberCallback);

//   ros::Publisher pub = n.advertise<std_msgs::String>("target_topic", 1000);
 
//   ros::Rate loop_rate(10);

//   int count = 0;
//   while (ros::ok())
//   {
//     std_msgs::String msg;
 
//     std::stringstream ss;
//     ss << "hello world " << count;
//     msg.data = ss.str();
 
//     ROS_INFO("%s", msg.data.c_str());
 
//     pub.publish(msg);
 
//     ros::spinOnce();
 
//     loop_rate.sleep();
//     ++count;
//   }
 
  return 0;
}
