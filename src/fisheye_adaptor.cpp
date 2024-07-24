#include "orb_slam2_ros/fisheye_adaptor.h"

#include <iostream>
using namespace std;

namespace orb_slam2_ros {

FisheyeUndistorter::FisheyeUndistorter(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport)
    : node_handle_(node_handle), image_transport_(image_transport) {
    node_name_ = ros::this_node::getName();

    load_fisheye_camera_parameters();
    image_subscriber_ = image_transport_.subscribe("/fisheye/image_raw", 1, &FisheyeUndistorter::callback_image, this);
    image_publisher_ = image_transport_.advertise("/fisheye/image_undistorted", 1);
}

FisheyeUndistorter::~FisheyeUndistorter() {}

void FisheyeUndistorter::callback_image(const sensor_msgs::ImageConstPtr &msg) {
    // convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        // TODO: encoding? sensor_msgs::image_encodings::MONO8
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
        cv_ptr = cv_bridge::toCvCopy(msg); 
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // undistort the image, maintain camera matrix K
    cv::Mat image_undistorted;
    cv::fisheye::undistortImage(cv_ptr->image, image_undistorted, K_, D_, K_, cv_ptr->image.size());

    // convert cv::Mat to sensor_msgs::Image
    cv_bridge::CvImage cv_image(msg->header, sensor_msgs::image_encodings::BGR8, image_undistorted);
    sensor_msgs::ImagePtr msg_undistorted = cv_image.toImageMsg();

    // publish the undistorted image
    image_publisher_.publish(msg_undistorted);
}

void FisheyeUndistorter::load_fisheye_camera_parameters() {
    // load camera parameters
    double fx, fy, cx, cy;
    double k1, k2, k3, k4;

    node_handle_.param<double>(node_name_+"/fx", fx, 381.36246688113556);
    node_handle_.param<double>(node_name_+"/fisheye/fy", fy, 381.36246688113556);
    node_handle_.param<double>(node_name_+"/fisheye/cx", cx, 320.5);
    node_handle_.param<double>(node_name_+"/fisheye/cy", cy, 240.5);
    node_handle_.param<double>(node_name_+"/fisheye/k1", k1, 0.0);
    node_handle_.param<double>(node_name_+"/fisheye/k2", k2, 0.0);
    node_handle_.param<double>(node_name_+"/fisheye/k3", k3, 0.0);
    node_handle_.param<double>(node_name_+"/fisheye/k4", k4, 0.0);

    K_ = (cv::Mat_<double>(3, 3) << 
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1);

    D_ = (cv::Mat_<double>(4, 1) << k1, k2, k3, k4);

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
}

} // namespace orb_slam2_ros

int main(int argc, char **argv) {
    ros::init(argc, argv, "fisheye_adaptor");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    orb_slam2_ros::FisheyeUndistorter fu(node_handle, image_transport);

    ros::spin();
    return 0;
}