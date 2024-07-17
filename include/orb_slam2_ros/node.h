#ifndef ORB_SLAM2_ROS_NODE_H_
#define ORB_SLAM2_ROS_NODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include "System.h"

namespace orb_slam2_ros {

class node {
    public:
        node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type);
        ~node();
        void initialize();
    
    protected:
        ORB_SLAM2::System *orb_slam_;
        void postprocess();
        void publish();

    private:
        ORB_SLAM2::System::eSensor sensor_type_;
        void initialize_ros_side();
        void initialize_orb_slam2();

        std::string node_name_;
        ros::NodeHandle node_handle_;
        image_transport::ImageTransport image_transport_;

        ros::Publisher map_publisher_;
        ros::Publisher pose_publisher_;
        image_transport::Publisher image_publisher_;

        ros::ServiceServer save_map_service_;

        bool publish_point_cloud_;
        bool publish_pose_;
        bool publish_tf_;
        bool publish_rendered_image_;
        bool load_map_;
        std::string map_file_name_;
};

} // namespace orb_slam2_ros

#endif // ORB_SLAM2_ROS_NODE_H_