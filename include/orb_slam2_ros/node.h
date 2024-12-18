#ifndef ORB_SLAM2_ROS_NODE_H_
#define ORB_SLAM2_ROS_NODE_H_

#include <ros/ros.h>

// packages
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// TF related
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// messages
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

// services
#include <orb_slam2_ros/SaveMap.h>
#include <orb_slam2_ros/SetLocalizationMode.h>
#include <orb_slam2_ros/RescaleMap.h>
#include <orb_slam2_ros/SetMopp.h>
#include <orb_slam2_ros/SetOffset.h>

// ORB-SLAM2
#include "System.h"

#define RESOLVE_POINT_CLOUD_ONTO_ORB_SLAM_INITIAL_FRAME_ENU
#undef RESOLVE_POINT_CLOUD_ONTO_ORB_SLAM_INITIAL_FRAME_ENU

#define SCALE_FACTOR_MIN 0.0F
#define SCALE_FACTOR_MAX 2.5e2F

#define DEFAULT_NAMESPACE "ifs"

namespace orb_slam2_ros {

// Frame convention: ORB-SLAM2=RDF, ROS=ENU, assuming initialliy facing east
/*
    Conversion from ORB_SLAM2 to ROS(ENU)
        ORB-SLAM2:  X-right,    Y-down,     Z-forward
        ROS ENU:    X-east,     Y-north,    Z-up

    but one is local frame (camera) and the other is inertial (world or map).. this is wierd
    Do we have to assume that the initial pose (of ORB_SLAM2) is identity? and aligned to ENU? -> yes, and thus FLU=ENU

    Transformation: 
        X_enu = Z_orb
        Y_enu = -X_orb
        Z_enu = -Y_orb
*/
const tf2::Matrix3x3 R_rdf_to_flu_  (0, 0, 1,
                                    -1, 0, 0,
                                    0,-1, 0);
const tf2::Matrix3x3 R_flu_to_rdf_  (R_rdf_to_flu_.transpose());
const tf2::Matrix3x3 R_flu_to_frd_  (1, 0, 0,
                                    0, -1, 0,
                                    0, 0, -1);
const tf2::Matrix3x3 R_frd_to_flu_  (R_flu_to_frd_.transpose());
const tf2::Matrix3x3 R_ned_to_enu_  (0, 1, 0,
                                    1, 0, 0,
                                    0, 0,-1);
const tf2::Matrix3x3 R_enu_to_ned_  (R_ned_to_enu_.transpose());

// relative pose of camera w.r.t body
const tf2::Matrix3x3 R_body_to_cam_ (1, 0, 0,
                                    0, 1, 0,
                                    0, 0, 1);
const tf2::Vector3 t_body_to_cam_   (-0.004, 0, 0.065);
const tf2::Matrix3x3 R_cam_to_body_ (R_body_to_cam_.transpose());
const tf2::Vector3 t_cam_to_body_   (R_cam_to_body_ * (-t_body_to_cam_));

class node {
    public:
        node(ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport, ORB_SLAM2::System::eSensor sensor_type);
        ~node();
    
    protected:
        // slam-derived
        std::mutex mutex_pose_;
        cv::Mat latest_Tcw_;
        ros::Time latest_image_time_internal_use_;
        ros::Time latest_image_time_linux_monotonic_; // time to be sent to MAVROS
        struct timespec ts; // for clock_gettime in linux, which will be relayed to MAVROS
        ORB_SLAM2::System *orb_slam_;
        bool slam_initialized_;

        void initialize_node();
        void check_slam_initialized(const int tracking_state);
        void update_latest_linux_monotonic_clock_time();
        void publish_rendered_image(cv::Mat image);
        void publish_point_cloud(std::vector<ORB_SLAM2::MapPoint*> map_points);

    private:
        void initialize_ros_side();
        void initialize_orb_slam2();
        void initialize_post_slam();
        bool load_orb_slam_parameters();
        bool load_orb_slam_parameters_from_topic();
        bool load_orb_slam_parameters_from_file(const std::string &filename);
        bool load_orb_slam_parameters_from_server();

        void publish_pose(const ros::TimerEvent&);

        bool service_save_map(orb_slam2_ros::SaveMap::Request &req, orb_slam2_ros::SaveMap::Response &res);
        bool service_set_localization_mode(orb_slam2_ros::SetLocalizationMode::Request &req, orb_slam2_ros::SetLocalizationMode::Response &res);
        bool service_rescale_map(orb_slam2_ros::RescaleMap::Request &req, orb_slam2_ros::RescaleMap::Response &res);
        bool service_set_minimum_observations_per_point(orb_slam2_ros::SetMopp::Request &req, orb_slam2_ros::SetMopp::Response &res);
        bool service_set_offset(orb_slam2_ros::SetOffset::Request &req, orb_slam2_ros::SetOffset::Response &res);

        tf2::Transform convert_orb_homogeneous_to_local_enu(cv::Mat Tcw); // to the initial frame of the ORB-SLAM2
        void update_local_tf();
        void print_transform_info(const tf2::Transform &tf, const std::string &name);
        bool load_initial_pose(const std::string &file_name);
        bool save_initial_pose(const std::string &file_name);

        // node
        std::string node_name_;
        std::string namespace_;
        ros::NodeHandle node_handle_;
        image_transport::ImageTransport image_transport_;

        // publisher & subscriber
        ros::Timer timer_; // periodic publication of pose
        ros::Publisher map_publisher_;
        ros::Publisher pose_publisher_visualization_;
        ros::Publisher pose_publisher_mavros_;
        image_transport::Publisher rendered_image_publisher_;

        // tf
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener tfListener_;

        // services & related variables
        ros::ServiceServer save_map_service_;
        ros::ServiceServer set_localization_mode_service_;
        ros::ServiceServer rescale_service_;
        ros::ServiceServer set_mopp_service_; // minimum observations per point
        ros::ServiceServer set_offset_service_; // user can additionally shift the origin
        std::vector<float> scale_factor_;
        int min_observations_per_point_;

        // ros flags
        bool publish_map_;
        bool publish_pose_;
        bool publish_tf_;
        bool publish_rendered_image_;

        // vehicle pose when ORB-SLAM2 is initialized
        tf2::Transform tf_map_to_vehicle_init_;
        tf2::Transform tf_vehicle_init_to_map_;
        tf2::Transform tf_user_offset_;
        tf2::Transform tf_user_offset_inverse_;

        // slam parameters
        std::string vocabulary_file_name_;
        ORB_SLAM2::System::eSensor sensor_type_;
        ORB_SLAM2::TrackingParameters orb_slam_tracking_parameters_;
        std::string map_file_name_;
        bool load_map_;
        std::string camera_info_topic_;
        bool camera_info_received_;

        // slam-derived
        int last_tracking_state_;
        tf2::Transform latest_local_tf_; // taking the initial frame as the reference frame
        tf2::Stamped<tf2::Transform> latest_stamped_local_tf_;
};

} // namespace orb_slam2_ros

#endif // ORB_SLAM2_ROS_NODE_H_