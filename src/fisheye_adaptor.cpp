#include "orb_slam2_ros/fisheye_adaptor.h"

#include <iostream>
#include <fstream>

using namespace std;

namespace orb_slam2_ros {

FisheyeUndistorter::FisheyeUndistorter(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport)
    : node_handle_(node_handle), image_transport_(image_transport), initialized_(false) {
    node_name_ = ros::this_node::getName();

    if (!load_fisheye_camera_parameters()) {
        ROS_ERROR("[Fisheye] Failed to load fisheye camera parameters. Terminating node.");
        ros::shutdown();
        return;
    }
    ROS_INFO("[Fisheye] Fisheye camera parameters are loaded.");
    print_loaded_parameters();
}

FisheyeUndistorter::~FisheyeUndistorter() {
    ROS_INFO("[Fisheye] Terminating FisheyeUndistorter.");
}

void FisheyeUndistorter::initialize() {
    image_publisher_ = image_transport_.advertise(node_name_+"/image_undistorted", 1);
    camera_info_publisher_ = node_handle_.advertise<sensor_msgs::CameraInfo>(node_name_+"/rectified_camera_info", 1);

    timer_ = node_handle_.createTimer(ros::Duration(0.5), &FisheyeUndistorter::callback_timer, this);
    image_subscriber_ = image_transport_.subscribe("/fisheye/image_raw", 1, &FisheyeUndistorter::callback_image, this);
}

void FisheyeUndistorter::callback_image(const sensor_msgs::ImageConstPtr &msg) {
    // convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        // encoding 
        // cv_ptr = cv_bridge::toCvCopy(msg); // follow the encoding of the input image
        // cv_prt = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // convert to MONO8
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (!initialized_) {
        initialize_undistortion_map(cv_ptr->image.size());
    }

    // undistort the image
    cv::Mat image_undistorted;
    cv::remap(cv_ptr->image, image_undistorted, undistortion_map1_, undistortion_map2_, cv::INTER_CUBIC, cv::BORDER_CONSTANT);

    // publish sensor_msgs::Image converted from cv::Mat(=image_undistorted)
    sensor_msgs::ImagePtr msg_undistorted = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image_undistorted).toImageMsg();
    image_publisher_.publish(msg_undistorted);
}

void FisheyeUndistorter::initialize_undistortion_map(const cv::Size& image_size) {
    // Resize the image and adjust the camera matrix
    node_handle_.param<double>(node_name_+"/resize_factor", resize_factor_, 2.0);
    image_size_undistorted_ = cv::Size(image_size.width/resize_factor_, image_size.height/resize_factor_);
    K_undistorted_ = K_.clone();
    K_undistorted_.at<float>(0, 0) /= resize_factor_; // fx
    K_undistorted_.at<float>(1, 1) /= resize_factor_; // fy
    K_undistorted_.at<float>(0, 2) /= resize_factor_; // cx
    K_undistorted_.at<float>(1, 2) /= resize_factor_; // cy

    // Initialize undistortion & rectification map
    cv::fisheye::initUndistortRectifyMap(
        K_, D_, cv::Matx33f::eye(), K_undistorted_, image_size_undistorted_, CV_32FC1, undistortion_map1_, undistortion_map2_);

    ROS_INFO("[Fisheye] Undistortion map initialized.");
    initialized_ = true;
}

bool FisheyeUndistorter::load_fisheye_camera_parameters() {
    // check if calibration file name is given
    std::string calibration_file_name;
    node_handle_.param<std::string>(node_name_+"/calibration_file", calibration_file_name, "");

    if (!calibration_file_name.empty() && load_fisheye_camera_parameters_from_file(calibration_file_name)) {
        return true;
    }

    // continue even if calibration file is not given or not correctly loaded from the file
    ROS_INFO("[Fisheye] No calibration file is loaded. Try loading from the server.");
    return load_fisheye_camera_parameters_from_server();
}

bool FisheyeUndistorter::load_fisheye_camera_parameters_from_file(const std::string &filename) {
    ROS_INFO("[Fisheye] Calibration file name: %s", filename.c_str());
    cv::FileStorage calibration_file(filename, cv::FileStorage::READ);
    if(!calibration_file.isOpened()){
        ROS_ERROR("[Fisheye] Failed to open file %s", filename.c_str());
        return false;
    }

    cv::FileNode camera_matrix_node = calibration_file["camera_matrix"];
    cv::FileNode data_node = camera_matrix_node["data"];
    K_ = cv::Mat(3, 3, CV_32F); // Ensure K_ is of correct size and type
    cv::FileNodeIterator it = data_node.begin();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j, ++it) {
            K_.at<float>(i, j) = static_cast<float>(*it);
        }
    }

    cv::FileNode distortion_coefficients_node = calibration_file["distortion_coefficients"];
    data_node = distortion_coefficients_node["data"];
    D_ = cv::Mat(1, 4, CV_32F); // Ensure D_ is of correct size and type
    it = data_node.begin();
    for (int i = 0; i < 4; ++i, ++it) {
        D_.at<float>(i) = static_cast<float>(*it);
    }
    calibration_file.release();
    return true;
}

bool FisheyeUndistorter::load_fisheye_camera_parameters_from_server() {
    bool loaded = true;
    float fx, fy, cx, cy;
    float k1, k2, k3, k4;

    loaded &= node_handle_.getParam(node_name_+"/fx", fx);
    loaded &= node_handle_.getParam(node_name_+"/fy", fy);
    loaded &= node_handle_.getParam(node_name_+"/cx", cx);
    loaded &= node_handle_.getParam(node_name_+"/cy", cy);
    loaded &= node_handle_.getParam(node_name_+"/k1", k1);
    loaded &= node_handle_.getParam(node_name_+"/k2", k2);
    loaded &= node_handle_.getParam(node_name_+"/k3", k3);
    loaded &= node_handle_.getParam(node_name_+"/k4", k4);
    if (!loaded) {
        ROS_ERROR("[Fisheye] Failed to load fisheye camera parameters from server.");
        return false;
    }

    K_ = (cv::Mat_<float>(3, 3) << 
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);

    D_ = (cv::Mat_<float>(1, 4) << k1, k2, k3, k4);
    return true;
}

void FisheyeUndistorter::callback_timer(const ros::TimerEvent&){
    // publish camera info message. periodically
    if (!initialized_) {
        return;
    }
    sensor_msgs::CameraInfo camera_info_msg;
    camera_info_msg.header.frame_id = "fisheye";
    camera_info_msg.width = image_size_undistorted_.width;
    camera_info_msg.height = image_size_undistorted_.height;

    // Initialize K with zeros
    std::fill(camera_info_msg.K.begin(), camera_info_msg.K.end(), 0.0);
    camera_info_msg.K[0] = K_undistorted_.at<float>(0, 0);
    camera_info_msg.K[2] = K_undistorted_.at<float>(0, 2);
    camera_info_msg.K[4] = K_undistorted_.at<float>(1, 1);
    camera_info_msg.K[5] = K_undistorted_.at<float>(1, 2);
    camera_info_msg.K[8] = 1;
    // Undistorted, thus no distortion coefficients
    camera_info_msg.D = {0.0, 0.0, 0.0, 0.0, 0.0};

    camera_info_publisher_.publish(camera_info_msg);
}

void FisheyeUndistorter::print_loaded_parameters() {
    ROS_INFO("[Fisheye] Camera parameters:");
    cout << "- fx: " << K_.at<float>(0, 0) << endl;
    cout << "- fy: " << K_.at<float>(1, 1) << endl;
    cout << "- cx: " << K_.at<float>(0, 2) << endl;
    cout << "- cy: " << K_.at<float>(1, 2) << endl;
    ROS_INFO("[Fisheye] Distortion coefficients:");
    cout << "- k1: " << D_.at<float>(0, 0) << endl;
    cout << "- k2: " << D_.at<float>(0, 1) << endl;
    cout << "- k3: " << D_.at<float>(0, 2) << endl;
    cout << "- k4: " << D_.at<float>(0, 3) << endl;
}

} // namespace orb_slam2_ros

int main(int argc, char **argv) {
    ros::init(argc, argv, "fisheye_adaptor");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    orb_slam2_ros::FisheyeUndistorter fisheye_undistorter(node_handle, image_transport);
    fisheye_undistorter.initialize();

    ros::spin();
    return 0;
}