#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace Ras{
geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id);
double quatToYaw(const geometry_msgs::Quaternion &in_quat);
float calcDistOfPoints(const geometry_msgs::Point &p_1, const geometry_msgs::Point &p_2);
}

struct RasVector
{
    float x;
    float y;
    float z;
    float len;
    RasVector(const geometry_msgs::Point &from, const geometry_msgs::Point &to);
    RasVector(const geometry_msgs::Pose &in_pose);
    RasVector(const geometry_msgs::Twist &in_twist);
};
