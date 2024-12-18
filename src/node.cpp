#include "orb_slam2_ros/node.h"
#include "orb_slam2_ros/boost_serialization.h"
#include <ros/package.h> // to get the package path

namespace orb_slam2_ros {

node::node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node_handle_(node_handle), image_transport_(image_transport), sensor_type_(sensor_type), tfListener_(tfBuffer_),
        slam_initialized_(false), scale_factor_(3, 1.0F), min_observations_per_point_(2), camera_info_received_(false) {
    node_name_ = ros::this_node::getName();
    namespace_ = ros::this_node::getNamespace();

    // initial tf
    latest_local_tf_ = tf2::Transform(tf2::Matrix3x3::getIdentity(), tf2::Vector3(0, 0, 0));
    tf_user_offset_ = tf2::Transform(tf2::Matrix3x3::getIdentity(), tf2::Vector3(0, 0, 0));
    tf_user_offset_inverse_ = tf_user_offset_.inverse();
    initialize_node();
}

node::~node() {
    ROS_INFO("[ORB_SLAM2_ROS] Terminating ORB-SLAM2 node.");
    orb_slam_->Shutdown();
    delete orb_slam_;
}

void node::initialize_node() {
    initialize_ros_side();
    initialize_orb_slam2();
    initialize_post_slam();
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
    node_handle_.param(node_name_+"/map_file_name", map_file_name_, std::string("map_file_name"));
    
    if(publish_map_) {
        map_publisher_ = node_handle_.advertise<sensor_msgs::PointCloud2>(node_name_+"/map", 1);
    }
    if(publish_rendered_image_) {
        rendered_image_publisher_ = image_transport_.advertise(node_name_+"/rendered_image", 1);
    }
    if(publish_pose_) {
        pose_publisher_visualization_ = node_handle_.advertise<geometry_msgs::PoseStamped>(node_name_+"/pose", 1);
        pose_publisher_mavros_ = node_handle_.advertise<geometry_msgs::PoseStamped>("/ifs/mavros/vision_pose/pose", 1);
    }
    timer_ = node_handle_.createTimer(ros::Duration(0.02), &node::publish_pose, this); // as fast as it can
}

void node::initialize_orb_slam2() {
    // initialize ORB-SLAM
    if (!load_orb_slam_parameters()){
        ROS_ERROR("[ORB_SLAM2_ROS] Parameters are not loaded correctly. Terminating node.");
        ros::shutdown();
        return;
    }

    // Turn localization mode ON by default, one should manually switch to mapping mode using dynamic reconfigure to build map
    orb_slam_ = new ORB_SLAM2::System(vocabulary_file_name_, sensor_type_, orb_slam_tracking_parameters_, map_file_name_ + ".bin", load_map_);
    orb_slam_->TurnLocalizationMode(false);
    last_tracking_state_ = orb_slam_->GetTrackingState();

    // load the initial vehicle pose if the map is given
    if (load_map_ && !load_initial_pose(map_file_name_)) {
        ROS_ERROR("[ORB_SLAM2_ROS] Initial vehicle pose is not loaded. Terminating node.");
        ros::shutdown();
        return;
    }
}

void node::initialize_post_slam() {
    // service server for saving map
    save_map_service_ = node_handle_.advertiseService(node_name_+"/save_map", &node::service_save_map, this);

    // service server for setting slam mode
    set_localization_mode_service_ = node_handle_.advertiseService(node_name_+"/set_localization_mode", &node::service_set_localization_mode, this);

    // service server for rescaling
    rescale_service_ = node_handle_.advertiseService(node_name_+"/rescale", &node::service_rescale_map, this);

    // service server for minimum observations per point
    set_mopp_service_ = node_handle_.advertiseService(node_name_+"/set_mopp", &node::service_set_minimum_observations_per_point, this);

    // service server for setting offset
    set_offset_service_ = node_handle_.advertiseService(node_name_+"/set_offset", &node::service_set_offset, this);
}

bool node::load_initial_pose(const std::string &file_name) {
    std::ifstream in(file_name + "_initial_tf.bin");
    if (!in.is_open()) {
        ROS_ERROR("[ORB_SLAM2_ROS] Initial tf file %s not found.", (file_name + "_initial_tf.bin").c_str());
        return false;
    }
    try {
        boost::archive::binary_iarchive ia(in, boost::archive::no_header);
        ia >> tf_map_to_vehicle_init_;
        ia >> tf_vehicle_init_to_map_;
        in.close();
        print_transform_info(tf_map_to_vehicle_init_, "Loaded initial vehicle pose");
        return true;
    } catch (...) {
        return false;
    }
}

void node::check_slam_initialized(const int tracking_state) {
    if (!slam_initialized_) {
        if (tracking_state == ORB_SLAM2::Tracking::eTrackingState::OK && last_tracking_state_ != ORB_SLAM2::Tracking::eTrackingState::OK) {
            if (!load_map_){
                // Get the initial vehicle pose, if the map is not given
                geometry_msgs::TransformStamped tf_voxl_to_map = tfBuffer_.lookupTransform("voxl", "map", ros::Time(0));
                tf2::fromMsg(tf_voxl_to_map.transform, tf_vehicle_init_to_map_);
                tf_map_to_vehicle_init_ = tf_vehicle_init_to_map_.inverse();

                print_transform_info(tf_map_to_vehicle_init_, "Initial vehicle pose");
            }
            slam_initialized_ = true;
            ROS_INFO("[ORB_SLAM2_ROS] SLAM initialized.");
        }
        last_tracking_state_ = tracking_state;
    }
}

void node::update_latest_linux_monotonic_clock_time() {
    clock_gettime(CLOCK_MONOTONIC, &ts);
    latest_image_time_linux_monotonic_.sec = ts.tv_sec;
    latest_image_time_linux_monotonic_.nsec = ts.tv_nsec;
}

bool node::load_orb_slam_parameters() {
    // load SLAM initialization parameters
    node_handle_.param(node_name_+"/vocabulary_file_name", vocabulary_file_name_, std::string("you can't run if you don't have one"));

    load_orb_slam_parameters_from_topic();

    // check if parameter file is given
    std::string config_file_name;
    node_handle_.param<std::string>(node_name_+"/parameter_file", config_file_name, "");
    if (!config_file_name.empty() && load_orb_slam_parameters_from_file(config_file_name)) {
        ROS_INFO("[ORB_SLAM2_ROS] Parameters are loaded from file.");
        return true;
    }

    // continue even if the parameter file is not given or parameters not correctly loaded from the file
    return load_orb_slam_parameters_from_server();
}

bool node::load_orb_slam_parameters_from_topic(){
    // camera info subscriber topic name
    node_handle_.param<std::string>(node_name_+"/camera_info_topic", camera_info_topic_, "");

    if (camera_info_topic_.empty()){
        ROS_INFO("[ORB_SLAM2_ROS] Camera info topic is not provided.");
        return false;
    }
    ROS_INFO("[ORB_SLAM2_ROS] Camera info topic: %s", camera_info_topic_.c_str());

    sensor_msgs::CameraInfo::ConstPtr camera_info_msg = 
        ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_, ros::Duration(20));
    if (camera_info_msg == nullptr){
        ROS_ERROR("[ORB_SLAM2_ROS] Camera info topic provided but message is not received.");
        return false;
    }
    
    camera_info_received_ = true;
    orb_slam_tracking_parameters_.fx = camera_info_msg->K[0];
    orb_slam_tracking_parameters_.fy = camera_info_msg->K[4];
    orb_slam_tracking_parameters_.cx = camera_info_msg->K[2];
    orb_slam_tracking_parameters_.cy = camera_info_msg->K[5];

    orb_slam_tracking_parameters_.k1 = camera_info_msg->D[0];
    orb_slam_tracking_parameters_.k2 = camera_info_msg->D[1];
    orb_slam_tracking_parameters_.p1 = camera_info_msg->D[2];
    orb_slam_tracking_parameters_.p2 = camera_info_msg->D[3];
    orb_slam_tracking_parameters_.k3 = camera_info_msg->D[4];

    ROS_INFO("[ORB_SLAM2_ROS] Camera parameters are loaded from topic.");
    return true;
}

bool node::load_orb_slam_parameters_from_file(const std::string &filename){
    cv::FileStorage config_file(filename, cv::FileStorage::READ);

    if(!config_file.isOpened()){
        ROS_ERROR("[ORB_SLAM2_ROS] Failed to open parameter file %s", filename.c_str());
        return false;
    }

    if(!camera_info_received_) {
        config_file["Camera.fx"] >> orb_slam_tracking_parameters_.fx;
        config_file["Camera.fy"] >> orb_slam_tracking_parameters_.fy;
        config_file["Camera.cx"] >> orb_slam_tracking_parameters_.cx;
        config_file["Camera.cy"] >> orb_slam_tracking_parameters_.cy;
        config_file["Camera.k1"] >> orb_slam_tracking_parameters_.k1;
        config_file["Camera.k2"] >> orb_slam_tracking_parameters_.k2;
        config_file["Camera.p1"] >> orb_slam_tracking_parameters_.p1;
        config_file["Camera.p2"] >> orb_slam_tracking_parameters_.p2;
        config_file["Camera.k3"] >> orb_slam_tracking_parameters_.k3;
    }
    config_file["Camera.fps"] >> orb_slam_tracking_parameters_.fps;
    config_file["Camera.RGB"] >> orb_slam_tracking_parameters_.isRGB;

    config_file["ORBextractor.nFeatures"] >> orb_slam_tracking_parameters_.nFeatures;
    config_file["ORBextractor.scaleFactor"] >> orb_slam_tracking_parameters_.scaleFactor;
    config_file["ORBextractor.nLevels"] >> orb_slam_tracking_parameters_.nLevels;
    config_file["ORBextractor.iniThFAST"] >> orb_slam_tracking_parameters_.iniThFAST;
    config_file["ORBextractor.minThFAST"] >> orb_slam_tracking_parameters_.minThFAST;

    if (sensor_type_ == ORB_SLAM2::System::STEREO || sensor_type_ == ORB_SLAM2::System::RGBD) {
        config_file["ThDepth"] >> orb_slam_tracking_parameters_.thDepth;
        config_file["Camera.bf"] >> orb_slam_tracking_parameters_.baseline;
        if (sensor_type_ == ORB_SLAM2::System::RGBD) {
            config_file["depthMapFactor"] >> orb_slam_tracking_parameters_.depthMapFactor;
        }
    }

    config_file.release();
    return true;
}

bool node::load_orb_slam_parameters_from_server(){
    bool loaded = true;
    // ORB parameters
    loaded &= node_handle_.getParam(node_name_+"/ORBextractor/nFeatures", orb_slam_tracking_parameters_.nFeatures);
    loaded &= node_handle_.getParam(node_name_+"/ORBextractor/scaleFactor", orb_slam_tracking_parameters_.scaleFactor);
    loaded &= node_handle_.getParam(node_name_+"/ORBextractor/nLevels", orb_slam_tracking_parameters_.nLevels);
    loaded &= node_handle_.getParam(node_name_+"/ORBextractor/iniThFAST", orb_slam_tracking_parameters_.iniThFAST);
    loaded &= node_handle_.getParam(node_name_+"/ORBextractor/minThFAST", orb_slam_tracking_parameters_.minThFAST);

    // camera parameters
    if (!camera_info_received_) {
        loaded &= node_handle_.getParam(node_name_+"/Camera/fx", orb_slam_tracking_parameters_.fx);
        loaded &= node_handle_.getParam(node_name_+"/Camera/fy", orb_slam_tracking_parameters_.fy);
        loaded &= node_handle_.getParam(node_name_+"/Camera/cx", orb_slam_tracking_parameters_.cx);
        loaded &= node_handle_.getParam(node_name_+"/Camera/cy", orb_slam_tracking_parameters_.cy);
        loaded &= node_handle_.getParam(node_name_+"/Camera/k1", orb_slam_tracking_parameters_.k1);
        loaded &= node_handle_.getParam(node_name_+"/Camera/k2", orb_slam_tracking_parameters_.k2);
        loaded &= node_handle_.getParam(node_name_+"/Camera/p1", orb_slam_tracking_parameters_.p1);
        loaded &= node_handle_.getParam(node_name_+"/Camera/p2", orb_slam_tracking_parameters_.p2);
        loaded &= node_handle_.getParam(node_name_+"/Camera/k3", orb_slam_tracking_parameters_.k3);
    }
    loaded &= node_handle_.getParam(node_name_+"/Camera/fps", orb_slam_tracking_parameters_.fps);
    loaded &= node_handle_.getParam(node_name_+"/Camera/rgb_encoding", orb_slam_tracking_parameters_.isRGB);

    // depth-involved
    if (sensor_type_ == ORB_SLAM2::System::STEREO || sensor_type_ == ORB_SLAM2::System::RGBD) {
        loaded &= node_handle_.getParam(node_name_+"/Camera/baseline", orb_slam_tracking_parameters_.baseline);
        loaded &= node_handle_.getParam(node_name_+"/ORBextractor/thDepth", orb_slam_tracking_parameters_.thDepth);
        loaded &= node_handle_.getParam(node_name_+"/ORBextractor/depthMapFactor", orb_slam_tracking_parameters_.depthMapFactor);
    }
    if (loaded) {
        ROS_INFO("[ORB_SLAM2_ROS] Parameters are loaded from server.");
    } else {
        ROS_ERROR("[ORB_SLAM2_ROS] Failed to load parameters from server.");
    }
    return loaded;
}

bool node::service_save_map(orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res){
    std::string file_name = ros::package::getPath("orb_slam2_ros") + "/resource/" + req.name;
    res.success = orb_slam_->SaveMap(file_name + ".bin");
    if (!res.success) {
        ROS_ERROR("[ORB_SLAM2_ROS] Map could not be saved.");
    } else if (!save_initial_pose(file_name)){
        ROS_ERROR("[ORB_SLAM2_ROS] Initial tf could not be saved.");
        res.success = false;
    } else {
        ROS_INFO("[ORB_SLAM2_ROS] Map & Initial pose saved.");
    }
    return res.success;
}

bool node::service_set_localization_mode(orb_slam2_ros::SetLocalizationMode::Request &req, orb_slam2_ros::SetLocalizationMode::Response &res){
    try {
        orb_slam_->TurnLocalizationMode(req.localization_mode);
    } catch (...) {
        res.success = false;
        return false;
    }
    res.success = (orb_slam_->GetLocalizationMode() == req.localization_mode);
    ROS_INFO("[ORB_SLAM2_ROS] %s mode %s", req.localization_mode ? "Localization" : "Mapping", res.success ? "set" : "not set");
    return res.success;
}

bool node::service_rescale_map(orb_slam2_ros::RescaleMap::Request &req, orb_slam2_ros::RescaleMap::Response &res){
    if (req.x >= SCALE_FACTOR_MIN && req.x <= SCALE_FACTOR_MAX) {
        scale_factor_[0] = req.x;
        ROS_INFO("[ORB_SLAM2_ROS] Map(X) rescaled by %.4f", scale_factor_[0]);
    } else {
        ROS_INFO("[ORB_SLAM2_ROS] Invalid scale factor (X): %.4f", req.x);
    }
    if (req.y >= SCALE_FACTOR_MIN && req.y <= SCALE_FACTOR_MAX) {
        scale_factor_[1] = req.y;
        ROS_INFO("[ORB_SLAM2_ROS] Map(Y) rescaled by %.4f", scale_factor_[1]);
    } else {
        ROS_INFO("[ORB_SLAM2_ROS] Invalid scale factor (Y): %.4f", req.y);
    }
    if (req.z >= SCALE_FACTOR_MIN && req.z <= SCALE_FACTOR_MAX) {
        scale_factor_[2] = req.z;
        ROS_INFO("[ORB_SLAM2_ROS] Map(Z) rescaled by %.4f", scale_factor_[2]);
    } else {
        ROS_INFO("[ORB_SLAM2_ROS] Invalid scale factor (Z) : %.4f", req.z);
    }
    res.success = true;
    return true;
}

bool node::service_set_minimum_observations_per_point(orb_slam2_ros::SetMopp::Request &req, orb_slam2_ros::SetMopp::Response &res){
    if (req.min_observations_per_point < 1) {
        return false;
    }
    if (req.min_observations_per_point > 10) {
        return false;
    }
    min_observations_per_point_ = req.min_observations_per_point;
    ROS_INFO("[ORB_SLAM2_ROS] Minimum observations per point set to %d", req.min_observations_per_point);
    res.success = true;
    return true;
}

bool node::service_set_offset(orb_slam2_ros::SetOffset::Request &req, orb_slam2_ros::SetOffset::Response &res){
    if (std::abs(req.origin.x) > 5e1F || std::abs(req.origin.y) > 5e1F || std::abs(req.origin.z) > 5e2F) {
        res.success = false;
        return false;
    }
    if (std::abs(req.euler_deg.x) > 30e1F || std::abs(req.euler_deg.y) > 30e1F || std::abs(req.euler_deg.z) > 30e1F) {
        res.success = false;
        return false;
    }
    tf2::Vector3 origin(req.origin.x, req.origin.y, req.origin.z);
    tf2::Quaternion rotation;
    rotation.setRPY(req.euler_deg.x*M_PI/180, req.euler_deg.y*M_PI/180, req.euler_deg.z*M_PI/180);

    tf_user_offset_.setOrigin(origin);
    tf_user_offset_.setRotation(rotation);
    tf_user_offset_inverse_ = tf_user_offset_.inverse();

    ROS_INFO("[ORB_SLAM2_ROS] User offset set: origin=(%.4f, %.4f, %.4f), euler=(%.2f, %.2f, %.2f)", 
        origin.x(), origin.y(), origin.z(), req.euler_deg.x, req.euler_deg.y, req.euler_deg.z);
    res.success = true;
    return true;
}

bool node::save_initial_pose(const std::string &file_name) {
    std::ofstream out(file_name + "_initial_tf.bin", std::ios_base::binary);
    if (!out.is_open()) {
        ROS_ERROR("[ORB_SLAM2_ROS] Cannot open Initial tf file %s.", (file_name + "_initial_tf.bin").c_str());
        return false;
    }
    try {
        boost::archive::binary_oarchive oa(out, boost::archive::no_header);
        oa << tf_map_to_vehicle_init_;
        oa << tf_vehicle_init_to_map_;
        out.close();
        print_transform_info(tf_map_to_vehicle_init_, "Saved initial vehicle pose");
        return true;
    } catch (...) {
        return false;
    }
}

void node::publish_rendered_image(cv::Mat image) {
    std_msgs::Header header;
    header.stamp = latest_image_time_internal_use_;
    header.frame_id = "tracking_camera";
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    rendered_image_publisher_.publish(image_msg);
}

void node::publish_pose(const ros::TimerEvent&) {
    if(!slam_initialized_ || !publish_pose_) {
        return;
    }
    update_local_tf();
    update_latest_linux_monotonic_clock_time();
    const tf2::Transform latest_global_tf_enu = tf_user_offset_inverse_ * tf_map_to_vehicle_init_ * latest_local_tf_;
    tf2::Stamped<tf2::Transform> latest_stamped_tf_visualization(latest_global_tf_enu, latest_image_time_internal_use_, "map");
    tf2::Stamped<tf2::Transform> latest_stamped_tf_mavros(latest_global_tf_enu, latest_image_time_linux_monotonic_, "map");

    geometry_msgs::PoseStamped pose_msg;
    pose_publisher_visualization_.publish(tf2::toMsg(latest_stamped_tf_visualization, pose_msg));
    pose_publisher_mavros_.publish(tf2::toMsg(latest_stamped_tf_mavros, pose_msg));

    if (publish_tf_) {
        geometry_msgs::TransformStamped latest_tf_stamped = tf2::toMsg(latest_stamped_tf_visualization);
        latest_tf_stamped.child_frame_id = namespace_.empty() ? DEFAULT_NAMESPACE : namespace_; // TODO: body to camera frame

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        tf_broadcaster.sendTransform(latest_tf_stamped);
    }
}

void node::update_local_tf() {
    std::lock_guard<std::mutex> lock(mutex_pose_);
    latest_local_tf_ = convert_orb_homogeneous_to_local_enu(latest_Tcw_);
}

void node::publish_point_cloud(std::vector<ORB_SLAM2::MapPoint*> map_points) {
    if (!publish_map_ || !slam_initialized_) {
        return;
    }
    if (map_points.empty()) {
        return;
    }
    sensor_msgs::PointCloud2 point_cloud;

    const int num_channels = 3;
    point_cloud.header.stamp = latest_image_time_internal_use_;
    point_cloud.header.frame_id = "map";;
    point_cloud.height = 1;
    point_cloud.width = map_points.size();
    point_cloud.is_bigendian = false;
    point_cloud.is_dense = true;
    point_cloud.point_step = num_channels * sizeof(float);
    point_cloud.row_step = point_cloud.point_step * point_cloud.width;
    point_cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        point_cloud.fields[i].name = channel_id[i];
        point_cloud.fields[i].offset = i * sizeof(float);
        point_cloud.fields[i].count = 1;
        point_cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    point_cloud.data.resize(point_cloud.row_step * point_cloud.height);

    float data_array[num_channels];
    unsigned char *cloud_data_ptr = &(point_cloud.data[0]);

#ifndef RESOLVE_POINT_CLOUD_ONTO_ORB_SLAM_INITIAL_FRAME_ENU
    tf2::Vector3 point;
    tf2::Vector3 point_transformed;
    tf2::Transform tf_map_to_vehicle_init_flu(tf_map_to_vehicle_init_.getBasis() * R_rdf_to_flu_, tf_map_to_vehicle_init_.getOrigin());
#endif

    for (unsigned int i = 0; i < map_points.size(); i++) {
        if (map_points[i] == nullptr) {
            continue;
        }
        if (map_points[i]->nObs >= min_observations_per_point_) {
            const cv::Mat point_world = map_points[i]->GetWorldPos();
            if (point_world.empty()) {
                continue;
            }
#ifdef RESOLVE_POINT_CLOUD_ONTO_ORB_SLAM_INITIAL_FRAME_ENU
            // RDF -> ENU (ORB-SLAM2 -> ROS), see convert_orb_homogeneous_to_local_enu
            data_array[0] = point_world.at<float>(2);
            data_array[1] = -point_world.at<float>(0);
            data_array[2] = -point_world.at<float>(1);
#else
            point.setValue(point_world.at<float>(0) * scale_factor_[0], 
                            point_world.at<float>(1) * scale_factor_[1], 
                            point_world.at<float>(2) * scale_factor_[2]);
            point_transformed = tf_user_offset_inverse_ * tf_map_to_vehicle_init_flu * point;
            data_array[0] = point_transformed.getX();
            data_array[1] = point_transformed.getY();
            data_array[2] = point_transformed.getZ();
#endif
            memcpy(cloud_data_ptr+(i*point_cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    map_publisher_.publish(point_cloud);
}

tf2::Transform node::convert_orb_homogeneous_to_local_enu(cv::Mat Tcw){
    if (Tcw.empty()) {
        return latest_local_tf_;
    }

    /*
    Worth noting: 
        1. the notation Rcw (or Tcw) converts "from world to camera"
        2. which is equivalent to C_n^b in the AE field, commonly denoted as Cnb. (n==w, b==c)
            (robotics)  R_cw * p_w = p_c 
            (AE)        C_w^c * p^w = p^c
    */

    // Convert rotation matrix to tf2::Matrix3x3
    tf2::Matrix3x3 tf2_Rcw(
        Tcw.at<float>(0, 0), Tcw.at<float>(0, 1), Tcw.at<float>(0, 2),
        Tcw.at<float>(1, 0), Tcw.at<float>(1, 1), Tcw.at<float>(1, 2),
        Tcw.at<float>(2, 0), Tcw.at<float>(2, 1), Tcw.at<float>(2, 2)
    );
    tf2::Vector3 tf2_tcw(Tcw.at<float>(0, 3) * scale_factor_[0], 
                            Tcw.at<float>(1, 3) * scale_factor_[1], 
                            Tcw.at<float>(2, 3) * scale_factor_[2]);
    tf2::Matrix3x3 tf2_Rwc = tf2_Rcw.transpose();
    tf2::Vector3 tf2_twc = -(tf2_Rwc * tf2_tcw);

    tf2::Matrix3x3 tf2_Rwb = R_cam_to_body_ * tf2_Rwc;
    tf2::Vector3 tf2_twb = R_cam_to_body_ * tf2_twc + t_cam_to_body_;

    // Rotation matrix should be left multiplied by R and right multiplied by its transpose, when resolving frame is changed
    return tf2::Transform(R_rdf_to_flu_ * tf2_Rwb * R_flu_to_rdf_, R_rdf_to_flu_ * tf2_twb);
}

void node::print_transform_info(const tf2::Transform &tf, const std::string &name) {
    tf2::Vector3 t = tf.getOrigin();

    double roll, pitch, yaw;
    tf.getBasis().getRPY(roll, pitch, yaw);

    ROS_INFO("[ORB_SLAM2_ROS] %s: x=%.4f, y=%.4f, z=%.4f", name.c_str(),
        t.getX(), t.getY(), t.getZ());
    ROS_INFO("[ORB_SLAM2_ROS] %s: roll=%.4f, pitch=%.4f, yaw=%.4f", name.c_str(), 
        roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
}

} // namespace orb_slam2_ros