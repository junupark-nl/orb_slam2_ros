#include "orb_slam2_ros/mono.h"

namespace orb_slam2_ros {

mono::mono(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type)
    : node(node_handle, image_transport, sensor_type) {
    // initialize Monocular ORB-SLAM
    image_subscriber_ = image_transport.subscribe("/camera/image_raw", 1, &mono::callback_image, this);
}

mono::~mono() {

}

void mono::callback_image(const sensor_msgs::ImageConstPtr &msg) {
    // convert sensor_msgs::Image to cv::Mat
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // mark the time of the last processed image
    latest_image_time_ = msg->header.stamp;
    // pass the image to ORB-SLAM
    latest_Tcw_ = orb_slam_->TrackMonocular(cv_ptr->image, msg->header.stamp.toSec());

    publish_topics();
}

} // namespace orb_slam2_ros

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orb_slam2_mono");
    ros::start();

    ros::NodeHandle node_handle;
    image_transport::ImageTransport image_transport(node_handle);

    orb_slam2_ros::mono slam_node(node_handle, image_transport, ORB_SLAM2::System::MONOCULAR);
    slam_node.initialize();

    ros::spin();
    return 0;
}
