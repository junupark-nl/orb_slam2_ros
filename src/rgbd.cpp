#include "orb_slam2_ros/rgbd.h"

namespace orb_slam2_ros {

rgbd::rgbd(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node(node_handle, image_transport, sensor_type) {
    // initialize RGBD ORB-SLAM

}

rgbd::~rgbd() {
    
}
    
} // namespace orb_slam2_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);
    orb_slam2_ros::rgbd slam_node(node_handle, image_transport, ORB_SLAM2::System::RGBD);

    slam_node.initialize();

    ros::spin();
    return 0;
}