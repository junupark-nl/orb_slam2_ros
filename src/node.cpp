#include "orb_slam2_ros/node.h"

node::node(ros::NodeHandle &node_handle) : node_handle_(node_handle) {
    // initialize ORB-SLAM
}

node::~node() {
    orb_slam_->Shutdown();
    delete orb_slam_;
}

void node::init() {
    // general initialization, such as:
    // subscribe to camera topic
    // advertise pose topic
    // advertise point cloud topic
    // advertise image topic
    // advertise camera info topic
    // advertise tf topic
}