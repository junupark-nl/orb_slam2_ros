#include "orb_slam2_ros/rgbd.h"

namespace orb_slam2_ros {

rgbd::rgbd(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node(node_handle, image_transport, sensor_type) {
    // initialize RGBD ORB-SLAM
    rgb_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/rgb/image_raw", 1);
    depth_image_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image>(node_handle, "/camera/depth_registered/image_raw", 1);

    synchronizer_ = new message_filters::Synchronizer<PolicyTimeSync>(PolicyTimeSync(10), *rgb_image_subscriber_, *depth_image_subscriber_);
    synchronizer_->registerCallback(boost::bind(&rgbd::callback_image, this, _1, _2));
}

rgbd::~rgbd() {
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
    latest_image_time_ = msg_rgb->header.stamp;
    // pass images to ORB-SLAM
    latest_Tcw_ = orb_slam_->TrackRGBD(cv_ptr_rgb->image, cv_ptr_depth->image, latest_image_time_.toSec());

    publish_topics();
}
    
} // namespace orb_slam2_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);
    orb_slam2_ros::rgbd slam_node(node_handle, image_transport, ORB_SLAM2::System::RGBD);

    slam_node.initialize();

    ros::spin();
    return 0;
}