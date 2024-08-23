#include "orb_slam2_ros/mono.h"

namespace orb_slam2_ros {

mono::mono(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node(node_handle, image_transport, sensor_type) {
    // Initiazte tracking callback of Monocular ORB-SLAM2
    image_subscriber_ = image_transport.subscribe("/camera/image_raw", 1, &mono::callback_image, this);

    // Set up a timer to publish point cloud
    timer_ = node_handle.createTimer(ros::Duration(2.0), &mono::callback_timer, this);
}

mono::~mono() {
    ROS_INFO("[Mono] Terminating Mono.");
}

void mono::callback_image(const sensor_msgs::ImageConstPtr &msg) {
    // convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // mark the time of the last processed image
    latest_image_time_internal_use_ = msg->header.stamp;
    update_latest_linux_monotonic_clock_time();

    // pass the image to ORB-SLAM
    latest_Tcw_ = orb_slam_->TrackMonocular(cv_ptr->image, latest_image_time_internal_use_.toSec());

    check_slam_initialized(orb_slam_->GetTrackingState());
    publish_rendered_image(orb_slam_->GetRenderedImage());
}

void mono::callback_timer(const ros::TimerEvent&) {
    publish_point_cloud(orb_slam_->GetAllMapPoints());
}

} // namespace orb_slam2_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    orb_slam2_ros::mono slam_node(node_handle, image_transport, ORB_SLAM2::System::MONOCULAR);

    ros::spin();
    return 0;
}