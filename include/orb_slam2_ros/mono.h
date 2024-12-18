#ifndef ORB_SLAM2_ROS_MONO_H_
#define ORB_SLAM2_ROS_MONO_H_

#include "orb_slam2_ros/node.h"

namespace orb_slam2_ros {

class mono: public node {
    public:
        mono(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type);
        ~mono();
    private:
        void callback_image(const sensor_msgs::ImageConstPtr &msg);
        void callback_timer(const ros::TimerEvent&);
        image_transport::Subscriber image_subscriber_;
        ros::Timer timer_;
};

} // namespace orb_slam2_ros

#endif // ORB_SLAM2_ROS_MONO_H_