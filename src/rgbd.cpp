#include "rgbd.h"

rgbd::rgbd(ros::NodeHandle &node_handle) : node(node_handle) {
    
}

rgbd::~rgbd() {
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::start();

    ros::NodeHandle node_handle;
    rgbd slam_node(node_handle);

    slam_node.init();

    ros::spin();
    return 0;
}