#ifndef FISHEYE_ADAPTOR_H_
#define FISHEYE_ADAPTOR_H_

#include <ros/ros.h>

// packages
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace orb_slam2_ros {

class FisheyeUndistorter {
    public:
        FisheyeUndistorter(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
        ~FisheyeUndistorter();
        void initialize();

    private:
        void callback_image(const sensor_msgs::ImageConstPtr &msg);
        void callback_timer(const ros::TimerEvent&);
        bool load_fisheye_camera_parameters();
        bool load_fisheye_camera_parameters_from_file(const std::string &filename);
        bool load_fisheye_camera_parameters_from_server();
        void print_loaded_parameters();
        void initialize_undistortion_map(const cv::Size& image_size);

        // node
        std::string node_name_;
        ros::NodeHandle node_handle_;
        image_transport::ImageTransport image_transport_;
        image_transport::Subscriber image_subscriber_;
        image_transport::Publisher image_publisher_;
        ros::Publisher camera_info_publisher_;
        ros::Timer timer_;

        // fisheye camera parameters
        cv::Mat K_;
        cv::Mat D_;

        // undistortion parameters
        bool initialized_;
        double resize_factor_;
        cv::Mat K_undistorted_;
        cv::Mat undistortion_map1_, undistortion_map2_;
        cv::Size image_size_undistorted_;
};

} // namespace orb_slam2_ros

#endif // FISHEYE_ADAPTOR_H_