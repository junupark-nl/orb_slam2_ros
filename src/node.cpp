#include "orb_slam2_ros/node.h"
#include <ros/package.h> // to get the package path

namespace orb_slam2_ros {

node::node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node_handle_(node_handle), image_transport_(image_transport), sensor_type_(sensor_type), tfListener_(tfBuffer_),
        initialized_(false), camera_info_received_(false) {
    node_name_ = ros::this_node::getName();
    namespace_ = ros::this_node::getNamespace();

    // default values of dynamically reconfigured parameters
    save_on_exit_ = false;
    min_observations_per_point_ = 2;
    dynamic_reconfigure_initial_setup_ = true;

    // initial tf
    latest_local_tf_ = tf2::Transform(tf2::Matrix3x3::getIdentity(), tf2::Vector3(0, 0, 0));

    initialize_node();
}

node::~node() {
    ROS_INFO("[ORB_SLAM2_ROS] Terminating ORB-SLAM2 node.");
    orb_slam_->Shutdown();
    if(save_on_exit_){
        orb_slam_->SaveTrajectoryTUM("trajectory.txt");
    }
    delete orb_slam_;
}

void node::initialize_node() {
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

    // camera info subscriber
    node_handle_.param<std::string>(node_name_+"/camera_info_topic", camera_info_topic_, "");
}

void node::initialize_orb_slam2() {
    // initialize ORB-SLAM
    if (!load_orb_slam_parameters()){
        ROS_ERROR("[ORB_SLAM2_ROS] Parameters are not loaded correctly. Terminating node.");
        ros::shutdown();
        return;
    }
    // Turn on localization mode by default, one should manually switch to mapping mode using dynamic reconfigure to build map
    orb_slam_ = new ORB_SLAM2::System(vocabulary_file_name_, sensor_type_, orb_slam_tracking_parameters_, map_file_name_, load_map_);
    orb_slam_->TurnLocalizationMode(true);
    last_tracking_state_ = orb_slam_->GetTrackingState();
}

void node::check_initialized(const int tracking_state) {
    if (!initialized_) {
        if (tracking_state == ORB_SLAM2::Tracking::eTrackingState::OK && last_tracking_state_ != ORB_SLAM2::Tracking::eTrackingState::OK) {
            // Get the initial vehicle pose
            geometry_msgs::TransformStamped transform_map_to_vehicle = tfBuffer_.lookupTransform("voxl", "map", ros::Time(0), ros::Duration(1.0));
            tf2::fromMsg(transform_map_to_vehicle.transform, tf_map_to_vehicle_init_);
            tf_vehicle_init_to_map_ = tf_map_to_vehicle_init_.inverse();
            // R_vehicle_init_to_map = tf_vehicle_init_to_map_.getBasis();
            // t_vehicle_init_to_map = tf_vehicle_init_to_map_.getOrigin();




            std::cout << "tf_map_to_vehicle_init_:" << std::endl;
            // Extract translation
            tf2::Vector3 translation = tf_map_to_vehicle_init_.getOrigin();
            
            // Extract rotation (quaternion)
            tf2::Quaternion rotation = tf_map_to_vehicle_init_.getRotation();
            
            // Convert quaternion to roll, pitch, yaw
            double roll, pitch, yaw;
            tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
            
            // Print using std::cout
            std::cout << "Translation: "
                    << "x=" << translation.x() << ", "
                    << "y=" << translation.y() << ", "
                    << "z=" << translation.z() << std::endl;
            std::cout << "Rotation (quaternion): "
                    << "x=" << rotation.x() << ", "
                    << "y=" << rotation.y() << ", "
                    << "z=" << rotation.z() << ", "
                    << "w=" << rotation.w() << std::endl;
            std::cout << "Rotation (RPY): "
                    << "roll=" << roll << ", "
                    << "pitch=" << pitch << ", "
                    << "yaw=" << yaw << std::endl;
            std::cout << "tf_vehicle_init_to_map_:" << std::endl;
            // Extract translation
            translation = tf_vehicle_init_to_map_.getOrigin();
            
            // Extract rotation (quaternion)
            rotation = tf_vehicle_init_to_map_.getRotation();
            
            // Convert quaternion to roll, pitch, yaw
            tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
            
            // Print using std::cout
            std::cout << "Translation: "
                    << "x=" << translation.x() << ", "
                    << "y=" << translation.y() << ", "
                    << "z=" << translation.z() << std::endl;
            std::cout << "Rotation (quaternion): "
                    << "x=" << rotation.x() << ", "
                    << "y=" << rotation.y() << ", "
                    << "z=" << rotation.z() << ", "
                    << "w=" << rotation.w() << std::endl;
            std::cout << "Rotation (RPY): "
                    << "roll=" << roll << ", "
                    << "pitch=" << pitch << ", "
                    << "yaw=" << yaw << std::endl;








            initialized_ = true;
            ROS_INFO("[ORB_SLAM2_ROS] SLAM initialized.");
        }
        last_tracking_state_ = tracking_state;
    }
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
        config_file["depthMapFactor"] >> orb_slam_tracking_parameters_.depthMapFactor;
        config_file["Camera.bf"] >> orb_slam_tracking_parameters_.baseline;
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
    std::string map_file_name = ros::package::getPath("orb_slam2_ros") + "/resource/" + req.name;
    res.success = orb_slam_->SaveMap(map_file_name);
    if (!res.success) {
        ROS_ERROR("[ORB_SLAM2_ROS] Map could not be saved.");
    }
    return res.success;
}

void node::reconfiguration_callback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level){
    if (dynamic_reconfigure_initial_setup_) {
        dynamic_reconfigure_initial_setup_ = false;
        return;
    }
    orb_slam_->TurnLocalizationMode(config.enable_localization_mode);
    save_on_exit_ = config.save_trajectory_on_exit;
    min_observations_per_point_ = config.min_observations_per_point;

    ROS_INFO("[ORB_SLAM2_ROS] reconfigured!");
    ROS_INFO("[ORB_SLAM2_ROS] - SLAM localization mode:\t%s", (config.enable_localization_mode ? "True" : "False"));
    ROS_INFO("[ORB_SLAM2_ROS] - save trajectory on exit:\t%s", (config.save_trajectory_on_exit ? "True" : "False")); 
    ROS_INFO("[ORB_SLAM2_ROS] - min observation points:\t%d", config.min_observations_per_point);
}

void node::publish_pose_and_image() {
    if(publish_rendered_image_) {
        publish_rendered_image(orb_slam_->GetRenderedImage());
    }
    if (!initialized_) {
        return;
    }
    if(publish_pose_) {
        publish_pose(latest_Tcw_);
    }
}

void node::publish_periodicals() {
    if (!initialized_) {
        return;
    }
    if (publish_map_) {
        publish_point_cloud(orb_slam_->GetAllMapPoints());
    }
}

void node::publish_rendered_image(cv::Mat image) {
    std_msgs::Header header;
    header.stamp = latest_image_time_;
    header.frame_id = "tracking_camera";
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    rendered_image_publisher_.publish(image_msg);
}

void node::publish_pose(cv::Mat Tcw) {
    update_local_tf();
    geometry_msgs::PoseStamped pose_msg;
    tf2::Stamped<tf2::Transform> latest_stamped_local_tf(latest_local_tf_ * tf_map_to_vehicle_init_, latest_image_time_, "map");
    pose_publisher_.publish(tf2::toMsg(latest_stamped_local_tf, pose_msg));

    if (publish_tf_) {
        geometry_msgs::TransformStamped latest_global_tf_stamped = tf2::toMsg(latest_stamped_local_tf);
        latest_global_tf_stamped.child_frame_id = namespace_.empty() ? DEFAULT_NAMESPACE : namespace_; // TODO: body to camera frame

        static tf2_ros::TransformBroadcaster tf_broadcaster;
        tf_broadcaster.sendTransform(latest_global_tf_stamped);
    }
}

void node::update_local_tf() {
    latest_local_tf_ = convert_orb_homogeneous_to_local_enu(latest_Tcw_);
}

void node::publish_point_cloud(std::vector<ORB_SLAM2::MapPoint*> map_points) {
    if (map_points.empty()) {
        return;
    }
    sensor_msgs::PointCloud2 point_cloud;

    const int num_channels = 3; // x y z
    point_cloud.header.stamp = latest_image_time_;
    point_cloud.header.frame_id = "map";;
    point_cloud.height = 1;
    point_cloud.width = map_points.size();
    point_cloud.is_bigendian = false;
    point_cloud.is_dense = true;
    point_cloud.point_step = num_channels * sizeof(float);
    point_cloud.row_step = point_cloud.point_step * point_cloud.width;
    point_cloud.fields.resize(num_channels);

    std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i<num_channels; i++) {
        point_cloud.fields[i].name = channel_id[i];
        point_cloud.fields[i].offset = i * sizeof(float);
        point_cloud.fields[i].count = 1;
        point_cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }
    point_cloud.data.resize(point_cloud.row_step * point_cloud.height);

    float data_array[num_channels];
    unsigned char *cloud_data_ptr = &(point_cloud.data[0]);
    for (unsigned int i=0; i<map_points.size(); i++) {
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
            data_array[0] = point_world.at<float>(0);
            data_array[1] = point_world.at<float>(1);
            data_array[2] = point_world.at<float>(2);
#endif
            memcpy(cloud_data_ptr+(i*point_cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }

#ifdef RESOLVE_POINT_CLOUD_ONTO_ORB_SLAM_INITIAL_FRAME_ENU
    map_publisher_.publish(point_cloud);
#else
    // tf2::Transform tf_camera_to_vehicle(R_enu_to_ned_ * R_cam_to_enu_); // in case tf_vehicle_init_to_map_ is ned
    tf2::Transform tf_camera_to_vehicle(R_cam_to_enu_); // currently don't bother with the translation
    tf2::Stamped<tf2::Transform> stamped_transform(tf_vehicle_init_to_map_ * tf_camera_to_vehicle, latest_image_time_, "map");

    sensor_msgs::PointCloud2 point_cloud_transformed;
    tf2::doTransform(point_cloud, point_cloud_transformed, tf2::toMsg(stamped_transform));
    map_publisher_.publish(point_cloud_transformed);
#endif
}

tf2::Transform node::convert_orb_homogeneous_to_local_enu(cv::Mat Tcw){
    /*
    Conversion from ORB_SLAM2 to ROS(ENU)
         ORB-SLAM2:  X-right,    Y-down,     Z-forward
         ROS ENU:    X-east,     Y-north,    Z-up
    Transformation: 
         X_enu = Z_orb
         Y_enu = -X_orb
         Z_enu = -Y_orb
    but one is local frame (camera) and the other is inertial (world or map).. this is wierd
    Do we have to assume that the initial pose (of ORB_SLAM2) is identity? and aligned to ENU? -> yes
    */

    if (Tcw.empty()) {
        return latest_local_tf_;
    }

    // Convert rotation matrix to tf2::Matrix3x3
    tf2::Matrix3x3 tf2_Rcw(
        Tcw.at<float>(0, 0), Tcw.at<float>(0, 1), Tcw.at<float>(0, 2),
        Tcw.at<float>(1, 0), Tcw.at<float>(1, 1), Tcw.at<float>(1, 2),
        Tcw.at<float>(2, 0), Tcw.at<float>(2, 1), Tcw.at<float>(2, 2)
    );
    tf2::Vector3 tf2_tcw(Tcw.at<float>(0, 3), Tcw.at<float>(1, 3), Tcw.at<float>(2, 3));
    tf2::Matrix3x3 tf2_Rwc = tf2_Rcw.transpose();
    tf2::Vector3 tf2_twc = -(tf2_Rwc * tf2_tcw);

    // Rotation matrix should be left multiplied by R_cam_to_enu_ and right multiplied by its transpose to get correct rotation
    return tf2::Transform(R_cam_to_enu_ * tf2_Rwc * R_cam_to_enu_.transpose(), R_cam_to_enu_ * tf2_twc);
}
} // namespace orb_slam2_ros