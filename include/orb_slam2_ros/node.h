#ifndef ORB_SLAM2_ROS_NODE_H_
#define ORB_SLAM2_ROS_NODE_H_

#include <ros/ros.h>

#include "System.h"

class node {
    public:
        node(ros::NodeHandle &node_handle);
        ~node();
        void init();
    
    protected:
        ros::NodeHandle node_handle_;
        ros::Publisher map_publisher_;

        ros::ServiceServer save_map_service_;

        ORB_SLAM2::System *orb_slam_;
};

#endif // ORB_SLAM2_ROS_NODE_H_