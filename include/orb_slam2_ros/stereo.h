#ifndef ORB_SLAM2_ROS_STEREO_H_
#define ORB_SLAM2_ROS_STEREO_H_

#include "orb_slam2_ros/node.h"

namespace orb_slam2_ros {

class stereo: public node {
    public:
        stereo(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type);
        ~stereo();
    private:
        image_transport::Subscriber image_subscriber_;
};

} // namespace orb_slam2_ros

#endif // ORB_SLAM2_ROS_STEREO_H_