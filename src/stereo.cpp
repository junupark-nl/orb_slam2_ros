#include "orb_slam2_ros/stereo.h"

stereo::stereo(ros::NodeHandle &node_handle) : node(node_handle) {

}

stereo::~stereo() {

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::start();

    ros::NodeHandle node_handle;
    stereo slam_node(node_handle);

    slam_node.init();

    ros::spin();
    return 0;
}