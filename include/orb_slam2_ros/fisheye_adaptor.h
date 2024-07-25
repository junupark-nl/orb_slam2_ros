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

    private:
        void callback_image(const sensor_msgs::ImageConstPtr &msg);
        void callback_timer(const ros::TimerEvent&);
        bool load_fisheye_camera_parameters();
        bool load_fisheye_camera_parameters_from_file(const std::string &filename);
        bool load_fisheye_camera_parameters_from_server();
        void print_loaded_parameters();
        void initialize_undistortion_map(const cv::Size& image_size);

        ros::NodeHandle node_handle_;
        image_transport::ImageTransport image_transport_;
        image_transport::Subscriber image_subscriber_;
        image_transport::Publisher image_publisher_;
        ros::Timer timer_;
        ros::Publisher camera_info_publisher_;
        std::string node_name_;

        cv::Mat K_;
        cv::Mat D_;
        cv::Size image_size_;
        bool initialized_;

        cv::Mat undistortion_map1_, undistortion_map2_;
};

}

#endif // FISHEYE_ADAPTOR_H_