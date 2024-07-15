#ifndef ORB_SLAM2_ROS_RGBD_H_
#define ORB_SLAM2_ROS_RGBD_H_

#include <ros/ros.h>

#include "orb_slam2_ros/node.h"

class rgbd: public node {
    public:
        rgbd(ros::NodeHandle &node_handle);
        ~rgbd();
};

#endif // ORB_SLAM2_ROS_RGBD_H_