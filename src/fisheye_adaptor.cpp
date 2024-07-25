#include "orb_slam2_ros/fisheye_adaptor.h"

#include <iostream>
#include <fstream>

using namespace std;

namespace orb_slam2_ros {

FisheyeUndistorter::FisheyeUndistorter(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport)
    : node_handle_(node_handle), image_transport_(image_transport), initialized_(false) {
    node_name_ = ros::this_node::getName();

    if (!load_fisheye_camera_parameters()) {
        ROS_ERROR("Failed to load fisheye camera parameters.");
        ros::shutdown();
        return;
    }
    ROS_INFO("Fisheye camera parameters are loaded.");
    image_subscriber_ = image_transport_.subscribe("/fisheye/image_raw", 1, &FisheyeUndistorter::callback_image, this);
    image_publisher_ = image_transport_.advertise("/fisheye/image_undistorted", 1);

    camera_info_publisher_ = node_handle_.advertise<sensor_msgs::CameraInfo>("/fisheye/rectified_camera_info", 1);
    timer_ = node_handle_.createTimer(ros::Duration(0.25), &FisheyeUndistorter::callback_timer, this);
}

FisheyeUndistorter::~FisheyeUndistorter() {}

void FisheyeUndistorter::callback_image(const sensor_msgs::ImageConstPtr &msg) {
    // convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        // TODO: encoding? sensor_msgs::image_encodings::MONO8
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        // cv_ptr = cv_bridge::toCvCopy(msg); 
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (!initialized_) {
        image_size_raw_ = cv_ptr->image.size();
        initialize_rectification();
        initialized_ = true;
    }

    // undistort the image
    cv::Mat image_undistorted;
    cv::fisheye::undistortImage(cv_ptr->image, image_undistorted, K_, D_, K_undistorted_, image_size_undistorted_);

    // convert cv::Mat to sensor_msgs::Image
    cv_bridge::CvImage cv_image(msg->header, sensor_msgs::image_encodings::BGR8, image_undistorted);
    sensor_msgs::ImagePtr msg_undistorted = cv_image.toImageMsg();

    // publish the undistorted image
    image_publisher_.publish(msg_undistorted);
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

    std::fill(camera_info_msg.K.begin(), camera_info_msg.K.end(), 0.0);
    camera_info_msg.K[0] = K_undistorted_.at<float>(0, 0);
    camera_info_msg.K[2] = K_undistorted_.at<float>(0, 2);
    camera_info_msg.K[4] = K_undistorted_.at<float>(1, 1);
    camera_info_msg.K[5] = K_undistorted_.at<float>(1, 2);
    camera_info_msg.K[8] = 1;

    camera_info_msg.D = {0.0, 0.0, 0.0, 0.0, 0.0};  // Initialize with zeros

    camera_info_publisher_.publish(camera_info_msg);
}

void FisheyeUndistorter::initialize_rectification() {
    // transfer maximum ROI to the undistorted image
    cv::Mat K_undistorted;
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K_, D_, image_size_raw_, cv::Matx33d::eye(), K_undistorted);

    cv::Mat map1, map2;
    cv::fisheye::initUndistortRectifyMap(K_, D_, cv::Matx33d::eye(), K_undistorted, image_size_raw_, CV_32FC1, map1, map2);
    
    // find valid ROI
    cv::Rect valid_roi_ = cv::Rect(0, 0, image_size_raw_.width, image_size_raw_.height);
    for(int y = 0; y < image_size_raw_.height; y++) 
    {
        for(int x = 0; x < image_size_raw_.width; x++) 
        {
            if(map1.at<float>(y,x) > 0 && map1.at<float>(y,x) < image_size_raw_.width &&
                map2.at<float>(y,x) > 0 && map2.at<float>(y,x) < image_size_raw_.height) 
            {
                valid_roi_.x = std::min(valid_roi_.x, x);
                valid_roi_.y = std::min(valid_roi_.y, y);
                valid_roi_.width = std::max(valid_roi_.width, x - valid_roi_.x + 1);
                valid_roi_.height = std::max(valid_roi_.height, y - valid_roi_.y + 1);
            }
        }
    }

    // Step 4: Adjust the camera matrix for the new ROI
    image_size_undistorted_ = valid_roi_.size();
    K_undistorted_ = K_undistorted.clone();
    K_undistorted_.at<float>(0,2) -= valid_roi_.x; // Adjust cx
    K_undistorted_.at<float>(1,2) -= valid_roi_.y; // Adjust cy
}

bool FisheyeUndistorter::load_fisheye_camera_parameters() {
    // check if calibration file name is given
    std::string calibration_file_name;
    node_handle_.param<std::string>(node_name_+"/calibration_file", calibration_file_name, "");

    if (!calibration_file_name.empty() && load_fisheye_camera_parameters_from_file(calibration_file_name)) {
        return true;
    }
    // continue even if calibration file is not given or not correctly loaded from the file
    return load_fisheye_camera_parameters_from_server();
}

bool FisheyeUndistorter::load_fisheye_camera_parameters_from_file(const std::string &filename) {
    
    ROS_INFO("calibration file name: %s", filename.c_str());
    cv::FileStorage calibration_file(filename, cv::FileStorage::READ);
    if(!calibration_file.isOpened()){
        ROS_ERROR("Failed to open file %s", filename.c_str());
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

    cout << "Fisheye camera parameters:" << endl;
    cout << "- fx: " << K_.at<float>(0, 0) << endl;
    cout << "- fy: " << K_.at<float>(1, 1) << endl;
    cout << "- cx: " << K_.at<float>(0, 2) << endl;
    cout << "- cy: " << K_.at<float>(1, 2) << endl;
    cout << "Distortion coefficients:" << endl;
    cout << "- k1: " << D_.at<float>(0, 0) << endl;
    cout << "- k2: " << D_.at<float>(0, 1) << endl;
    cout << "- k3: " << D_.at<float>(0, 2) << endl;
    cout << "- k4: " << D_.at<float>(0, 3) << endl;
    return true;
}

bool FisheyeUndistorter::load_fisheye_camera_parameters_from_server() {

    bool loaded = true;
    // load camera parameters
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
        ROS_ERROR("Failed to load fisheye camera parameters from server.");
        return false;
    }

    K_ = (cv::Mat_<float>(3, 3) << 
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);

    D_ = (cv::Mat_<float>(1, 4) << k1, k2, k3, k4);

    cout << "Fisheye camera parameters:" << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "Distortion coefficients:" << endl;
    cout << "- k1: " << k1 << endl;
    cout << "- k2: " << k2 << endl;
    cout << "- k3: " << k3 << endl;
    cout << "- k4: " << k4 << endl;
    return true;
}

} // namespace orb_slam2_ros

int main(int argc, char **argv) {
    ros::init(argc, argv, "fisheye_adaptor");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    orb_slam2_ros::FisheyeUndistorter fisheye_undistorter(node_handle, image_transport);

    ros::spin();
    return 0;
}