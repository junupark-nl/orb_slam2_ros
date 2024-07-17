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
    // parameter initialization
    node_handle_.param("publish_point_cloud", publish_point_cloud_, false);
    node_handle_.param("publish_rendered_image", publish_rendered_image_, false);
    node_handle_.param("publish_pose", publish_pose_, false);
    node_handle_.param("publish_tf", publish_tf_, false);

    node_handle_.param("load_map", load_map_, false);
    node_handle_.param("map_file_name", map_file_name_, std::string("map.bin"));
    
    if(publish_point_cloud_) {
        // advertise point cloud topic
    }
    if(publish_rendered_image_) {
        // advertise image topic
    }
    if(publish_pose_) {
        // advertise pose topic
    }
    if(publish_tf_) {
        // advertise tf topic
    }

    // dynamic reconfigure
    // - tracking mode or not on the fly

    // service server for saving map
}

void node::initialize_orb_slam2() {
    // initialize ORB-SLAM
    ORB_SLAM2::TrackingParameters tracking_parameters;
    // TODO: set parameters based on ROS params
    orb_slam_ = new ORB_SLAM2::System("vocabulary", sensor_type_, tracking_parameters, map_file_name_, load_map_);
}

void node::postprocess() {
    // parse information of orb_slam_, whenever it is updated via image callback, into easy-usable form
}

void node::publish() {
    const auto map_points = orb_slam_->GetAllMapPoints();

    if(publish_point_cloud_) {
        // publish point cloud
    }
    if(publish_rendered_image_) {
        // publish image
    }
    if(publish_pose_) {
        // publish pose
    }
    if(publish_tf_) {
        // publish tf
    }
}
    
} // namespace orb_slam2_ros