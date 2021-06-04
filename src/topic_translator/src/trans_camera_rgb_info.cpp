#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/CameraInfo.h"

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

    }

    void init(int argc, char **argv, std::string node_name)
    {
        ros::init(argc, argv, node_name);
        n_ = new ros::NodeHandle();
    }

    void setSubTopic(std::string topic)
    {
        sub_ = n_->subscribe(topic, 1000, this->SubscriberCallback);
    }

    void setPubTopic(std::string topic)
    {
        pub_ = n_->advertise<sensor_msgs::CameraInfo>(topic, 1000);
    }

    void startTrans()
    {
        ros::spin();
    }

private:
    void static SubscriberCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
	    pThis->pub_.publish(msg);
        ROS_INFO("I heard: [%s]", msg->header);
    }
    
private:
    static TopicTranslator* pThis;

    ros::NodeHandle* n_;

    ros::Subscriber sub_;
    ros::Publisher pub_;
};

TopicTranslator* TopicTranslator::pThis = nullptr;

///arm_camera/depth_registered/camera_info
///arm_camera/depth_registered/image_raw
///arm_camera/rgb/camera_info
///arm_camera/rgb/image_raw

///camera/depth/camera_info
///camera/depth/image_raw
///camera/rgb/camera_info
///camera/rgb/image_raw

int main(int argc, char **argv)
{
    TopicTranslator topic_translator;

    topic_translator.init(argc, argv, "trans_camera_rgb_info");

    topic_translator.setSubTopic("arm_camera/rgb/camera_info");

    topic_translator.setPubTopic("camera/rgb/camera_info");

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
