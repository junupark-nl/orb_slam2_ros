#include "orb_slam2_ros/rgbd.h"

namespace orb_slam2_ros {

rgbd::rgbd(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node(node_handle, image_transport, sensor_type) {
    // Initiazte tracking callback of RGBD ORB-SLAM2
    rgb_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/rgb/image_raw", 1);
    depth_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/depth_registered/image_raw", 1);

    synchronizer_ = new message_filters::Synchronizer<PolicyTimeSync>(PolicyTimeSync(10), *rgb_image_subscriber_, *depth_image_subscriber_);
    synchronizer_->registerCallback(boost::bind(&rgbd::callback_image, this, _1, _2));

    // Set up a timer to publish point cloud
    timer_ = node_handle.createTimer(ros::Duration(2.0), &rgbd::callback_timer, this);
}

rgbd::~rgbd() {
    ROS_INFO("[RGBD] Terminating RGBD.");
    delete rgb_image_subscriber_;
    delete depth_image_subscriber_;
    delete synchronizer_;
}

void rgbd::callback_image(const sensor_msgs::ImageConstPtr &msg_rgb, const sensor_msgs::ImageConstPtr &msg_depth) {
    // convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr_rgb, cv_ptr_depth;
    try {
        cv_ptr_rgb = cv_bridge::toCvShare(msg_rgb);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    try {
        cv_ptr_depth = cv_bridge::toCvShare(msg_depth);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // mark the time of the last processed image
    latest_image_time_internal_use_ = msg_rgb->header.stamp;
    update_latest_linux_monotonic_clock_time();

    // pass images to ORB-SLAM
    latest_Tcw_ = orb_slam_->TrackRGBD(cv_ptr_rgb->image, cv_ptr_depth->image, latest_image_time_internal_use_.toSec());

    check_slam_initialized(orb_slam_->GetTrackingState());
    publish_rendered_image(orb_slam_->GetRenderedImage());
}

void rgbd::callback_timer(const ros::TimerEvent&) {
    publish_point_cloud(orb_slam_->GetAllMapPoints());
}

} // namespace orb_slam2_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_rgbd");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);
    
    orb_slam2_ros::rgbd slam_node(node_handle, image_transport, ORB_SLAM2::System::RGBD);

    ros::spin();
    return 0;
}