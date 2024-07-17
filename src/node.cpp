#include "orb_slam2_ros/node.h"

namespace orb_slam2_ros {

node::node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node_handle_(node_handle), image_transport_(image_transport), sensor_type_(sensor_type) {
    // initialize ORB-SLAM
}

node::~node() {
    orb_slam_->Shutdown();
    delete orb_slam_;
}

void node::initialize() {
    this->initialize_ros_side();
    this->initialize_orb_slam2();
}

void node::initialize_ros_side() {
    // general initialization, such as:
    // advertise pose topic
    // advertise point cloud topic
    // advertise image topic
    // advertise camera info topic
    // advertise tf topic
    // configure based on ROS params
}

void node::initialize_orb_slam2() {
    // initialize ORB-SLAM
    ORB_SLAM2::ORBParameters orb_parameters;
    // TODO: set parameters based on ROS params
    orb_slam_ = new ORB_SLAM2::System("vocabulary", sensor_type_, orb_parameters);
}

void node::postprocess() {
    // parse information of orb_slam_, whenever it is updated via image callback, into easy-usable form
}

void node::publish() {
    // publish pose
    // publish point cloud
    // publish image
    // publish camera info
    // publish tf
}
    
} // namespace orb_slam2_ros