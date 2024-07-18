#ifndef ORB_SLAM2_ROS_NODE_H_
#define ORB_SLAM2_ROS_NODE_H_

#include <ros/ros.h>

// packages
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// TF related
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// Synchronization of a pair of stereo (or RGBD) images
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
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
        // slam-derived
        cv::Mat latest_Tcw_;
        ros::Time latest_image_time_;
        ORB_SLAM2::System *orb_slam_;
        void publish_topics();

    private:
        void initialize_ros_side();
        void initialize_orb_slam2();
        void load_orb_slam_parameters();

        void publish_point_cloud(std::vector<ORB_SLAM2::MapPoint*> map_points);
        void publish_rendered_image(cv::Mat image);
        void publish_pose(cv::Mat Tcw);
        void update_tf();
        tf2::Transform convert_orb_pose_to_tf(cv::Mat Tcw);

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

        // dynamic reconfigure & dynamically configured parameters
        dynamic_reconfigure::Server<orb_slam2_ros::dynamic_reconfigureConfig> dynamic_reconfigure_server_;
        bool save_on_exit_;
        int min_observations_per_point_;

        // ros flags
        bool publish_map_;
        bool publish_pose_;
        bool publish_tf_;
        bool publish_rendered_image_;

        // slam parameters
        std::string vocabulary_file_name_;
        ORB_SLAM2::System::eSensor sensor_type_;
        ORB_SLAM2::TrackingParameters orb_slam_tracking_parameters_;
        std::string map_file_name_;
        bool load_map_;

        // slam-derived
        tf2::Transform latest_tf_;
        tf2::Stamped<tf2::Transform> latest_tf_stamped_;
};

} // namespace orb_slam2_ros

#endif // ORB_SLAM2_ROS_NODE_H_