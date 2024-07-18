#include "orb_slam2_ros/node.h"

namespace orb_slam2_ros {

node::node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node_handle_(node_handle), image_transport_(image_transport), sensor_type_(sensor_type) {
    node_name_ = ros::this_node::getName();
}

node::~node() {
    orb_slam_->Shutdown();
    if(save_on_exit_){
        orb_slam_->SaveTrajectoryTUM("trajectory.txt");
    }
    delete orb_slam_;
}

void node::initialize() {
    initialize_ros_side();
    initialize_orb_slam2();
}

void node::initialize_ros_side() {
    // Publish flags
    node_handle_.param(node_name_+"/publish_map", publish_map_, false);
    node_handle_.param(node_name_+"/publish_rendered_image", publish_rendered_image_, false);
    node_handle_.param(node_name_+"/publish_pose", publish_pose_, false);
    node_handle_.param(node_name_+"/publish_tf", publish_tf_, false);
    // Map loading parameters
    node_handle_.param(node_name_+"/load_map", load_map_, false);
    node_handle_.param(node_name_+"/map_file_name", map_file_name_, std::string("map.bin"));
    
    if(publish_map_) {
        map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(node_name_+"/map", 1);
    }
    if(publish_rendered_image_) {
        rendered_image_publisher_ = image_transport_.advertise(node_name_+"/rendered_image", 1);
    }
    if(publish_pose_) {
        pose_publisher_ = node_handle_.advertise<geometry_msgs::PoseStamped>(node_name_+"/pose", 1);
    }

    // dynamic reconfigure
    dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig>::CallbackType dynamic_reconfigure_callback;
    dynamic_reconfigure_callback = boost::bind(&node::reconfiguration_callback, this, _1, _2);
    dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback);

    // service server for saving map
    save_map_service_ = node_handle_.advertiseService(node_name_+"/save_map", &node::service_save_map, this);
}

void node::initialize_orb_slam2() {
    // initialize ORB-SLAM
    load_orb_slam_parameters();
    orb_slam_ = new ORB_SLAM2::System(vocabulary_file_name_, sensor_type_, orb_slam_tracking_parameters_, map_file_name_, load_map_);
    save_on_exit_ = false;
}

void node::load_orb_slam_parameters() {
    // load SLAM initialization parameters
    // vocabulary file name
    node_handle_.param(node_name_+"/vocabulary_file_name", vocabulary_file_name_, std::string("you can't run if you don't have one"));

    // ORB parameters
    node_handle_.param(node_name_+"/ORB/nFeatures", orb_slam_tracking_parameters_.nFeatures, 1200);
    node_handle_.param(node_name_+"/ORB/scaleFactor", orb_slam_tracking_parameters_.scaleFactor, 1.2F);
    node_handle_.param(node_name_+"/ORB/nLevels", orb_slam_tracking_parameters_.nLevels, 8);
    node_handle_.param(node_name_+"/ORB/iniThFAST", orb_slam_tracking_parameters_.iniThFAST, 20);
    node_handle_.param(node_name_+"/ORB/minThFAST", orb_slam_tracking_parameters_.minThFAST, 7);

    // camera parameters
    node_handle_.param(node_name_+"/camera/fx", orb_slam_tracking_parameters_.fx, 520.9F);
    node_handle_.param(node_name_+"/camera/fy", orb_slam_tracking_parameters_.fy, 521.0F);
    node_handle_.param(node_name_+"/camera/cx", orb_slam_tracking_parameters_.cx, 325.1F);
    node_handle_.param(node_name_+"/camera/cy", orb_slam_tracking_parameters_.cy, 249.7F);
    node_handle_.param(node_name_+"/camera/k1", orb_slam_tracking_parameters_.k1, 0.2624F);
    node_handle_.param(node_name_+"/camera/k2", orb_slam_tracking_parameters_.k2, -0.9531F);
    node_handle_.param(node_name_+"/camera/p1", orb_slam_tracking_parameters_.p1, -0.0054F);
    node_handle_.param(node_name_+"/camera/p2", orb_slam_tracking_parameters_.p2, 0.0026F);
    node_handle_.param(node_name_+"/camera/k3", orb_slam_tracking_parameters_.k3, 1.1633F);
    node_handle_.param(node_name_+"/camera/fps", orb_slam_tracking_parameters_.fps, 30);
    node_handle_.param(node_name_+"/camera/rgb_encoding", orb_slam_tracking_parameters_.isRGB, true);

    // depth-involved
    if (sensor_type_ == ORB_SLAM2::System::STEREO || sensor_type_ == ORB_SLAM2::System::RGBD) {
        node_handle_.param(node_name_+"/camera/baseline", orb_slam_tracking_parameters_.baseline, 0.12F);
        node_handle_.param(node_name_+"/ORBextractor/thDepth", orb_slam_tracking_parameters_.thDepth, 35.0F);
        node_handle_.param(node_name_+"/ORBextractor/depthMapFactor", orb_slam_tracking_parameters_.depthMapFactor, 1.0F);
    }
}

bool node::service_save_map(orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res){
    res.success = orb_slam_->SaveMap(req.name);
    if (!res.success) {
        ROS_ERROR("Map could not be saved.");
    }
    return res.success;
}

void node::reconfiguration_callback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level){
    orb_slam_->TurnLocalizationMode(config.enable_localization_mode);
    save_on_exit_ = config.save_trajectory_on_exit;
}

void node::publish() {

    if(publish_map_) {
        publish_point_cloud(orb_slam_->GetAllMapPoints());
    }
    if(publish_rendered_image_) {
        publish_rendered_image(orb_slam_->GetRenderedImage());
    }
    if(publish_pose_) {
        publish_pose(orb_slam_->GetCurrentPoseCvMat());
    }
    if(publish_tf_) {
        publish_tf(orb_slam_->GetCurrentPoseCvMat());
    }
}

void node::publish_point_cloud(std::vector<ORB_SLAM2::MapPoint*> map_points) {
    // TODO: convert map_points to point cloud
    // publish point cloud
}

void node::publish_rendered_image(cv::Mat image) {
    // TODO: convert cv::Mat to sensor_msgs::Image
    // publish rendered image
}

void node::publish_pose(cv::Mat pose) {
    // TODO: convert cv::Mat to geometry_msgs::PoseStamped
    // publish pose
}

void node::publish_tf(cv::Mat pose) {
    // static tf2_ros::TransformBroadcaster tf_broadcater;
    // TODO: convert cv::Mat to tf
    // publish tf
    // geometry_msgs::TransformStamped transformStamped = tf2::toMsg(tf);
    // tf_broadcaster.sendTransform();
}
} // namespace orb_slam2_ros