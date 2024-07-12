#ifndef ORB_SLAM2_ROS_MONO_H_
#define ORB_SLAM2_ROS_MONO_H_

#include <ros/ros.h>

#include "node.h"

class mono: public node {
    public:
        mono(ros::NodeHandle &node_handle);
        ~mono();
}

#endif // ORB_SLAM2_ROS_MONO_H_