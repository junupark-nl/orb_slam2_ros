#ifndef ORB_SLAM2_ROS_STEREO_H_
#define ORB_SLAM2_ROS_STEREO_H_

#include <ros/ros.h>

#include "node.h"

class stereo: public node {
    public:
        stereo(ros::NodeHandle &node_handle);
        ~stereo();
}

#endif // ORB_SLAM2_ROS_STEREO_H_