#include <ros/ros.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <string>
#include <vector>

#define PI 3.14159265358979

using namespace sensor_msgs;
using namespace message_filters;
using namespace nav_msgs;

class GSM_Topic_Sync
{
public:
    GSM_Topic_Sync()
    {
        sub_queue_size_ = 10;
        pub_rate_ = 1000;

        robot_num_ = 1;
        
        setPubTopic();
    }

    ~GSM_Topic_Sync()
    {

    }

    void setPubTopic()
    {
        camera_depth_camera_info_pub_ = nh.advertise<CameraInfo>("camera/depth/camera_info", pub_rate_);
        camera_depth_image_raw_pub_ = nh.advertise<Image>("camera/depth/image_raw", pub_rate_);
        camera_rgb_camera_info_pub_ = nh.advertise<CameraInfo>("camera/rgb/camera_info", pub_rate_);
        camera_rgb_image_raw_pub_ = nh.advertise<Image>("camera/rgb/image_raw", pub_rate_);
    }

    void setRobotNum(int robot_num)
    {
        if(robot_num > 0)
        {
            robot_num_ = robot_num;
        }
    }

    void startSync()
    {
        //0 : TFOnly
        //1 : ApproximateTime
        //2 : TimeSynchronizer
        int sync_mode = 0;

        if(sync_mode == 0)
        {
            for(int robot_idx = 0; robot_idx < robot_num_; ++robot_idx)
            {
                ros::Subscriber camera_ground_truth_sub;
                camera_ground_turth_sub_vec_.emplace_back(camera_ground_truth_sub);

                camera_ground_turth_sub_vec_[robot_idx] = nh.subscribe<Odometry>("srobot" + std::to_string(robot_idx) + "_camera_ground_truth", 1000,
                                                                                 [this, robot_idx](const auto& msg){ this->tfOnlyCallback(msg, robot_idx); });
            }

            ros::spin();
        }
        else if(sync_mode == 1)
        {
            message_filters::Subscriber<CameraInfo> camera_depth_camera_info_sub_(nh, "camera0/depth/camera_info", sub_queue_size_);
            message_filters::Subscriber<Image> camera_depth_image_raw_sub_(nh, "camera0/depth/image_raw", sub_queue_size_);
            message_filters::Subscriber<CameraInfo> camera_rgb_camera_info_sub_(nh, "camera0/rgb/camera_info", sub_queue_size_);
            message_filters::Subscriber<Image> camera_rgb_image_raw_sub_(nh, "camera0/rgb/image_raw", sub_queue_size_);
            message_filters::Subscriber<Odometry> camera_ground_truth_sub_(nh, "camera0_ground_truth", sub_queue_size_);
            
            typedef sync_policies::ApproximateTime<CameraInfo, Image, CameraInfo, Image, Odometry> MySyncPolicy;
            Synchronizer<MySyncPolicy> sync(MySyncPolicy(sub_queue_size_),
                                            camera_depth_camera_info_sub_,
                                            camera_depth_image_raw_sub_,
                                            camera_rgb_camera_info_sub_,
                                            camera_rgb_image_raw_sub_,
                                            camera_ground_truth_sub_);
            
            sync.registerCallback(boost::bind(&GSM_Topic_Sync::unionCallback, this, _1, _2, _3, _4, _5));

            ros::spin();
        }
        else if(sync_mode == 2)
        {
            message_filters::Subscriber<CameraInfo> camera_depth_camera_info_sub_(nh, "camera0/depth/camera_info", sub_queue_size_);
            message_filters::Subscriber<Image> camera_depth_image_raw_sub_(nh, "camera0/depth/image_raw", sub_queue_size_);
            message_filters::Subscriber<CameraInfo> camera_rgb_camera_info_sub_(nh, "camera0/rgb/camera_info", sub_queue_size_);
            message_filters::Subscriber<Image> camera_rgb_image_raw_sub_(nh, "camera0/rgb/image_raw", sub_queue_size_);
            message_filters::Subscriber<Odometry> camera_ground_truth_sub_(nh, "camera0_ground_truth", sub_queue_size_);

            TimeSynchronizer<CameraInfo, Image, CameraInfo, Image, Odometry> sync(camera_depth_camera_info_sub_,
                                                                                            camera_depth_image_raw_sub_,
                                                                                            camera_rgb_camera_info_sub_,
                                                                                            camera_rgb_image_raw_sub_,
                                                                                            camera_ground_truth_sub_,
                                                                                            sub_queue_size_);

            sync.registerCallback(boost::bind(&GSM_Topic_Sync::unionCallback, this, _1, _2, _3, _4, _5));
            
            ros::spin();
        }
    }

private:
    void tfOnlyCallback(const OdometryConstPtr& camera_ground_truth,
                        int robot_idx)
    {
        double camera_position_x = camera_ground_truth->pose.pose.position.x;
        double camera_position_y = camera_ground_truth->pose.pose.position.y;
        double camera_position_z = camera_ground_truth->pose.pose.position.z;

        tf2::Quaternion q_map_to_baselink;
        tf2::convert(camera_ground_truth->pose.pose.orientation, q_map_to_baselink);

        tf2::Quaternion q_baselink_to_camera;
        q_baselink_to_camera.setEuler(PI / 2.0, 0, -PI / 2.0);

        geometry_msgs::TransformStamped transformStamped_map_to_baselink;
        transformStamped_map_to_baselink.header.frame_id = "map";
        transformStamped_map_to_baselink.child_frame_id = "srobot" + std::to_string(robot_idx) + "_base_link";
        transformStamped_map_to_baselink.transform.translation.x = camera_position_x;
        transformStamped_map_to_baselink.transform.translation.y = camera_position_y;
        transformStamped_map_to_baselink.transform.translation.z = camera_position_z;
        transformStamped_map_to_baselink.transform.rotation.x = q_map_to_baselink.x();
        transformStamped_map_to_baselink.transform.rotation.y = q_map_to_baselink.y();
        transformStamped_map_to_baselink.transform.rotation.z = q_map_to_baselink.z();
        transformStamped_map_to_baselink.transform.rotation.w = q_map_to_baselink.w();
        transformStamped_map_to_baselink.header.stamp = camera_ground_truth->header.stamp;

        geometry_msgs::TransformStamped transformStamped_baselink_to_camera;
        transformStamped_baselink_to_camera.header.frame_id = "srobot" + std::to_string(robot_idx) + "_base_link";
        transformStamped_baselink_to_camera.child_frame_id = "srobot" + std::to_string(robot_idx) + "_camera_frame";
        transformStamped_baselink_to_camera.transform.translation.x = 0;
        transformStamped_baselink_to_camera.transform.translation.y = 0;
        transformStamped_baselink_to_camera.transform.translation.z = 0;
        transformStamped_baselink_to_camera.transform.rotation.x = q_baselink_to_camera.x();
        transformStamped_baselink_to_camera.transform.rotation.y = q_baselink_to_camera.y();
        transformStamped_baselink_to_camera.transform.rotation.z = q_baselink_to_camera.z();
        transformStamped_baselink_to_camera.transform.rotation.w = q_baselink_to_camera.w();
        transformStamped_baselink_to_camera.header.stamp = camera_ground_truth->header.stamp;
        tf_pub_.sendTransform(transformStamped_map_to_baselink);
        tf_pub_.sendTransform(transformStamped_baselink_to_camera);

        // const std::string output_msg = "tfOnlyCallback : I heard srobot" + std::to_string(robot_idx) + "'s sync msgs.";

        // ROS_INFO(output_msg.c_str());
    }

    void unionCallback(const CameraInfoConstPtr& camera_depth_camera_info,
                       const ImageConstPtr& camera_depth_image_raw,
                       const CameraInfoConstPtr& camera_rgb_camera_info,
                       const ImageConstPtr& camera_rgb_image_raw,
                       const OdometryConstPtr& camera_ground_truth)
    {
        double camera_position_x = camera_ground_truth->pose.pose.position.x;
        double camera_position_y = camera_ground_truth->pose.pose.position.y;
        double camera_position_z = camera_ground_truth->pose.pose.position.z;

        tf2::Quaternion q_map_to_baselink;
        tf2::convert(camera_ground_truth->pose.pose.orientation, q_map_to_baselink);

        tf2::Quaternion q_baselink_to_camera;
        q_baselink_to_camera.setEuler(PI / 2.0, 0, -PI / 2.0);

        geometry_msgs::TransformStamped transformStamped_map_to_baselink;
        transformStamped_map_to_baselink.header.frame_id = "map";
        transformStamped_map_to_baselink.child_frame_id = "base_link";
        transformStamped_map_to_baselink.transform.translation.x = camera_position_x;
        transformStamped_map_to_baselink.transform.translation.y = camera_position_y;
        transformStamped_map_to_baselink.transform.translation.z = camera_position_z;
        transformStamped_map_to_baselink.transform.rotation.x = q_map_to_baselink.x();
        transformStamped_map_to_baselink.transform.rotation.y = q_map_to_baselink.y();
        transformStamped_map_to_baselink.transform.rotation.z = q_map_to_baselink.z();
        transformStamped_map_to_baselink.transform.rotation.w = q_map_to_baselink.w();
        transformStamped_map_to_baselink.header.stamp = camera_depth_image_raw->header.stamp;

        geometry_msgs::TransformStamped transformStamped_baselink_to_camera;
        transformStamped_baselink_to_camera.header.frame_id = "base_link";
        transformStamped_baselink_to_camera.child_frame_id = "camera0_frame";
        transformStamped_baselink_to_camera.transform.translation.x = 0;
        transformStamped_baselink_to_camera.transform.translation.y = 0;
        transformStamped_baselink_to_camera.transform.translation.z = 0;
        transformStamped_baselink_to_camera.transform.rotation.x = q_baselink_to_camera.x();
        transformStamped_baselink_to_camera.transform.rotation.y = q_baselink_to_camera.y();
        transformStamped_baselink_to_camera.transform.rotation.z = q_baselink_to_camera.z();
        transformStamped_baselink_to_camera.transform.rotation.w = q_baselink_to_camera.w();
        transformStamped_baselink_to_camera.header.stamp = camera_depth_image_raw->header.stamp;
        tf_pub_.sendTransform(transformStamped_map_to_baselink);
        tf_pub_.sendTransform(transformStamped_baselink_to_camera);

        camera_depth_camera_info_pub_.publish(camera_depth_camera_info);
        camera_depth_image_raw_pub_.publish(camera_depth_image_raw);
        camera_rgb_camera_info_pub_.publish(camera_rgb_camera_info);
        camera_rgb_image_raw_pub_.publish(camera_rgb_image_raw);

        // ROS_INFO("UnionCallback : I heard sync msgs.");
    }

public:
    ros::NodeHandle nh;
    
private:
    int sub_queue_size_;
    int pub_rate_;

    std::vector<ros::Subscriber> camera_ground_turth_sub_vec_;
    
    ros::Publisher camera_depth_camera_info_pub_;
    ros::Publisher camera_depth_image_raw_pub_;
    ros::Publisher camera_rgb_camera_info_pub_;
    ros::Publisher camera_rgb_image_raw_pub_;
    tf2_ros::TransformBroadcaster tf_pub_;

    int robot_num_;
};
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "gsm_topic_sync");

    GSM_Topic_Sync gsm_topic_sync;

    gsm_topic_sync.setRobotNum(atoi(argv[1]));

    gsm_topic_sync.startSync();

    return 0;
}
