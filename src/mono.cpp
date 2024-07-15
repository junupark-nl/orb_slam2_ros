#include "orb_slam2_ros/mono.h"

mono::mono(ros::NodeHandle &node_handle) : node(node_handle) {
    
}

mono::~mono() {

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::start();

    ros::NodeHandle node_handle;
    mono slam_node(node_handle);

    slam_node.init();

    ros::spin();
    return 0;
}