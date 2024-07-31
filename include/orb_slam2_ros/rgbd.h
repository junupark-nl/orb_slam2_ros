#ifndef ORB_SLAM2_ROS_RGBD_H_
#define ORB_SLAM2_ROS_RGBD_H_

#include "orb_slam2_ros/node.h"

// Synchronization of a pair of images (e.g. rgb & depth)
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace orb_slam2_ros {

class rgbd: public node {
    public:
        rgbd(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type);
        ~rgbd();
    private:
        void callback_image(const sensor_msgs::ImageConstPtr &msg_rgb, const sensor_msgs::ImageConstPtr &msg_depth);
        void callback_timer(const ros::TimerEvent&);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> PolicyTimeSync;
        message_filters::Synchronizer<PolicyTimeSync> *synchronizer_;
        message_filters::Subscriber<sensor_msgs::Image> *rgb_image_subscriber_;
        message_filters::Subscriber<sensor_msgs::Image> *depth_image_subscriber_;
        ros::Timer timer_;
};

} // namespace orb_slam2_ros

#endif // ORB_SLAM2_ROS_RGBD_H_