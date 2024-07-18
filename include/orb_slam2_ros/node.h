#ifndef ORB_SLAM2_ROS_NODE_H_
#define ORB_SLAM2_ROS_NODE_H_

#include <ros/ros.h>
// packages
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

// Synchronization of a pair of stereo (or RGBD) images
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
// TF related
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <orb_slam2_ros/SaveMap.h> // service
#include <orb_slam2_ros/dynamic_reconfigureConfig.h> // dynamic reconfigure

#include "System.h"

namespace orb_slam2_ros {

class node {
    public:
        node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type);
        ~node();
        void initialize();
    
    protected:
        ORB_SLAM2::System *orb_slam_;
        void publish();

    private:
        ORB_SLAM2::System::eSensor sensor_type_;
        void initialize_ros_side();
        void initialize_orb_slam2();
        void load_orb_slam_parameters();

        void publish_point_cloud(std::vector<ORB_SLAM2::MapPoint*> map_points);
        void publish_rendered_image(cv::Mat image);
        void publish_pose(cv::Mat pose);
        void publish_tf(cv::Mat pose);

        bool service_save_map(orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);
        void reconfiguration_callback(orb_slam2_ros::dynamic_reconfigureConfig &config, uint32_t level);

        // node
        std::string node_name_;
        ros::NodeHandle node_handle_;
        image_transport::ImageTransport image_transport_;

        // publisher
        ros::Publisher map_publisher_;
        ros::Publisher pose_publisher_;
        image_transport::Publisher rendered_image_publisher_;

        // tf
        boost::shared_ptr<tf2_ros::Buffer> tfBuffer;
        boost::shared_ptr<tf2_ros::TransformListener> tfListener;

        // service
        ros::ServiceServer save_map_service_;

        // dynamic reconfigure
        dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_reconfigure_server_;

        // ros parameters
        bool publish_map_;
        bool publish_pose_;
        bool publish_tf_;
        bool publish_rendered_image_;
        bool load_map_;
        bool save_on_exit_;
        std::string map_file_name_;
        std::string vocabulary_file_name_;
        ORB_SLAM2::TrackingParameters orb_slam_tracking_parameters_;
};

} // namespace orb_slam2_ros

#endif // ORB_SLAM2_ROS_NODE_H_