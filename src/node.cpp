#include "orb_slam2_ros/node.h"

namespace orb_slam2_ros {

node::node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node_handle_(node_handle), image_transport_(image_transport), sensor_type_(sensor_type) {
    node_name_ = ros::this_node::getName();
    // default values of dynamically reconfigured parameters
    save_on_exit_ = false;
    min_observations_per_point_ = 2;
    dynamic_reconfigure_initial_setup_ = true;
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
    if (publish_tf_ && !publish_pose_) { // you simply don't publish tf without pose
        publish_pose_ = true;
    }
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
        node_handle_.param(node_name_+"/ORB/thDepth", orb_slam_tracking_parameters_.thDepth, 35.0F);
        node_handle_.param(node_name_+"/ORB/depthMapFactor", orb_slam_tracking_parameters_.depthMapFactor, 1.0F);
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
    if (dynamic_reconfigure_initial_setup_) {
        dynamic_reconfigure_initial_setup_ = false;
    } else {
        orb_slam_->TurnLocalizationMode(config.enable_localization_mode);
    }
    save_on_exit_ = config.save_trajectory_on_exit;
    min_observations_per_point_ = config.min_observations_per_point;
    ROS_INFO("ORB_SLAM2_ROS reconfiguration.\r\n\t-SLAM localization mode:\t%s\r\n\t-save trajectory on exit:\t%s\r\n\t-min observation points:\t%d", 
        config.enable_localization_mode ? "True" : "False", 
        config.save_trajectory_on_exit ? "True" : "False", 
        config.min_observations_per_point);
}

void node::publish_topics() {
    if(publish_rendered_image_) {
        publish_rendered_image(orb_slam_->GetRenderedImage());
    }
    if(publish_pose_) {
        publish_pose(latest_Tcw_);
    }
    if(publish_map_) {
        publish_point_cloud(orb_slam_->GetAllMapPoints());
    }
}

void node::publish_rendered_image(cv::Mat image) {
    std_msgs::Header header;
    header.stamp = latest_image_time_;
    // TODO: TF-stuffs
    header.frame_id = "map";
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    rendered_image_publisher_.publish(image_msg);
}

void node::publish_pose(cv::Mat Tcw) {
    // TODO: TF-stuffs
    update_tf();
    geometry_msgs::PoseStamped pose_msg;
    pose_publisher_.publish(tf2::toMsg(latest_tf_stamped_, pose_msg));

    if (publish_tf_) {
        geometry_msgs::TransformStamped tf_msg = tf2::toMsg(latest_tf_stamped_);
        // TODO: TF-stuffs
        tf_msg.child_frame_id = "ifs";

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        tf_broadcaster.sendTransform(tf_msg);
    }
}

void node::update_tf() {
    latest_tf_ = convert_orb_pose_to_tf(latest_Tcw_);
    latest_tf_stamped_ = tf2::Stamped<tf2::Transform>(latest_tf_, latest_image_time_, "map");
}

void node::publish_point_cloud(std::vector<ORB_SLAM2::MapPoint*> map_points) {
    // TODO: convert map_points to point cloud correctly via transform
    if (map_points.empty()) {
        return;
    }
    sensor_msgs::PointCloud2 point_cloud;

    const int num_channels = 3; // x y z
    point_cloud.header.stamp = latest_image_time_;
    // TODO: TF-stuffs
    point_cloud.header.frame_id = "map";;
    point_cloud.height = 1;
    point_cloud.width = map_points.size();
    point_cloud.is_bigendian = false;
    point_cloud.is_dense = true;
    point_cloud.point_step = num_channels * sizeof(float);
    point_cloud.row_step = point_cloud.point_step * point_cloud.width;
    point_cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};
    for (int i = 0; i<num_channels; i++) {
        point_cloud.fields[i].name = channel_id[i];
        point_cloud.fields[i].offset = i * sizeof(float);
        point_cloud.fields[i].count = 1;
        point_cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    point_cloud.data.resize(point_cloud.row_step * point_cloud.height);

    unsigned char *cloud_data_ptr = &(point_cloud.data[0]);

    float data_array[num_channels];
    for (unsigned int i=0; i<map_points.size(); i++) {
        if (map_points[i] == nullptr) {
            continue;
        }
        if (map_points[i]->nObs >= min_observations_per_point_) {
            const cv::Mat point_world = map_points[i]->GetWorldPos();
            // TODO: rotation? frame convention?
            data_array[0] = point_world.at<float>(0);
            data_array[1] = point_world.at<float>(1);
            data_array[2] = point_world.at<float>(2);

            memcpy(cloud_data_ptr+(i*point_cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }

    if(!publish_pose_) {
        update_tf();
    }
    
    sensor_msgs::PointCloud2 point_cloud_transformed;
    tf2::doTransform(point_cloud, point_cloud_transformed, tf2::toMsg(latest_tf_stamped_));
    map_publisher_.publish(point_cloud_transformed);
}

tf2::Transform node::convert_orb_pose_to_tf(cv::Mat Tcw){
    if (Tcw.empty()) {
        return tf2::Transform(tf2::Matrix3x3::getIdentity(), tf2::Vector3(0, 0, 0));
    }
    cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
    cv::Mat tcw = Tcw.rowRange(0,3).col(3);

    cv::Mat Rwc = Rcw.t();
    cv::Mat twc = -Rwc*tcw;

    tf2::Matrix3x3 Rwc_tf;
    Rwc_tf.setValue(Rwc.at<float>(0, 0), Rwc.at<float>(0, 1), Rwc.at<float>(0, 2),
                    Rwc.at<float>(1, 0), Rwc.at<float>(1, 1), Rwc.at<float>(1, 2),
                    Rwc.at<float>(2, 0), Rwc.at<float>(2, 1), Rwc.at<float>(2, 2));
    tf2::Vector3 twc_tf(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

    const tf2::Matrix3x3 R_orb_to_tf(0, 0, 1,
                                      -1, 0, 0,
                                      0, -1, 0);

    return tf2::Transform(R_orb_to_tf * Rwc_tf, R_orb_to_tf * twc_tf);

    // Conversion from ORB_SLAM2 to ROS(ENU)
    //      ORB-SLAM2:  X-right,    Y-down,     Z-forward
    //      ROS ENU:    X-east,     Y-north,    Z-up
    // Transformation: 
    //      X_enu = Z_orb
    //      Y_enu = -X_orb
    //      Z_enu = -Y_orb
    // but one is local frame (camera) and the other is inertial (world or map).. this is wierd
    // Do we have to assume that initial pose (of ORB_SLAM2) is identity? and aligned to ENU?
}
} // namespace orb_slam2_ros