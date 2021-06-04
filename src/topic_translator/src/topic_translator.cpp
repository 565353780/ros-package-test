#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "tf/transform_broadcaster.h"
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

    }

    void setSubTopic()
    {
        camera_depth_camera_info_sub_ = n_.subscribe("arm_camera/depth_registered/camera_info", 1000, this->CameraDepthCameraInfoSubCallback);
        camera_depth_image_raw_sub_ = n_.subscribe("arm_camera/depth_registered/image_raw", 1000, this->CameraDepthImageRawSubCallback);
        camera_rgb_camera_info_sub_ = n_.subscribe("arm_camera/rgb/camera_info", 1000, this->CameraRgbCameraInfoSubCallback);
        camera_rgb_image_raw_sub_ = n_.subscribe("arm_camera/rgb/image_raw", 1000, this->CameraRgbImageRawSubCallback);
    }

    void setPubTopic()
    {
        camera_depth_camera_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info", 1000);
        camera_depth_image_raw_pub_ = n_.advertise<sensor_msgs::CameraInfo>("camera/depth/image_raw", 1000);
        camera_rgb_camera_info_pub_ = n_.advertise<sensor_msgs::CameraInfo>("camera/rgb/camera_info", 1000);
        camera_rgb_image_raw_pub_ = n_.advertise<sensor_msgs::CameraInfo>("camera/rgb/image_raw", 1000);
    }

    void startTrans()
    {
        ros::spin();
    }

private:
    void sendAllMsgs()
    {

    }

    void static CameraDepthCameraInfoSubCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        // pThis->camera_depth_camera_info_ = msg;
        pThis->sendAllMsgs();
        ROS_INFO("CameraDepthCameraInfoSubCallback : I heard");
    }
    void static CameraDepthImageRawSubCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
	    // pThis->camera_depth_image_raw_ = msg;
        pThis->sendAllMsgs();
        ROS_INFO("CameraDepthImageRawSubCallback : I heard");
    }
    void static CameraRgbCameraInfoSubCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
    {
        // pThis->camera_rgb_camera_info_ = msg;
        pThis->sendAllMsgs();
        ROS_INFO("CameraRgbCameraInfoSubCallback : I heard");
    }
    void static CameraRgbImageRawSubCallback(const sensor_msgs::Image::ConstPtr& msg)
    {
        // pThis->camera_rgb_image_raw_ = msg;
        pThis->sendAllMsgs();
        ROS_INFO("CameraRgbImageRawSubCallback : I heard");
    }
    
private:
    static TopicTranslator* pThis;

    ros::NodeHandle n_;

    ros::Subscriber camera_depth_camera_info_sub_;
    ros::Subscriber camera_depth_image_raw_sub_;
    ros::Subscriber camera_rgb_camera_info_sub_;
    ros::Subscriber camera_rgb_image_raw_sub_;
    
    ros::Publisher camera_depth_camera_info_pub_;
    ros::Publisher camera_depth_image_raw_pub_;
    ros::Publisher camera_rgb_camera_info_pub_;
    ros::Publisher camera_rgb_image_raw_pub_;

    sensor_msgs::CameraInfo camera_depth_camera_info_;
    sensor_msgs::Image camera_depth_image_raw_;
    sensor_msgs::CameraInfo camera_rgb_camera_info_;
    sensor_msgs::Image camera_rgb_image_raw_;

    bool camera_depth_camera_info_ready_;
    bool camera_depth_image_raw_ready_;
    bool camera_rgb_camera_info_ready_;
    bool camera_rgb_image_raw_ready_;
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
    ros::init(argc, argv, "topic_translator");
    
    TopicTranslator topic_translator;

    topic_translator.setSubTopic();

    topic_translator.setPubTopic();

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
