#include <ros/ros.h>
#include "ras_lib.h"

geometry_msgs::Pose Ras::tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
{
    static tf::TransformListener tf_listener;

    tf::Pose current_tf;
    tf::StampedTransform transform;
    tf::Pose transformed_tf;
    geometry_msgs::Pose transformed_pose;
    geometry_msgs::PoseStamped transform_origin;

    try{
        tf_listener.waitForTransform(current_frame_id, target_frame_id,  ros::Time(0), ros::Duration(1.0));
        // current_frame_id　から　target_frame_id　への座標変換
        tf_listener.lookupTransform(target_frame_id, current_frame_id, ros::Time(0), transform);

    }catch (tf::TransformException &ex)  {
        ROS_ERROR("%s", ex.what());
        //ros::Duration(1.0).sleep();
    }

    tf::poseMsgToTF(current_pose, current_tf);
    transformed_tf = transform * current_tf;
    tf::poseTFToMsg(transformed_tf, transformed_pose);

    return transformed_pose;
}

double Ras::quatToYaw(const geometry_msgs::Quaternion &in_quat)
{
    double roll, pitch, yaw;

    tf::Quaternion tf_quat(in_quat.x, in_quat.y, in_quat.z, in_quat.w);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

float Ras::calcDistOfPoints(const geometry_msgs::Point &p_1, const geometry_msgs::Point &p_2)
{
    return sqrt(pow(p_1.x - p_2.x, 2) + pow(p_1.y - p_2.y, 2));
}

CalcVector::CalcVector(const geometry_msgs::Point &from, const geometry_msgs::Point &to)
{
    x = to.x - from.x;
    y = to.y - from.y;
    z = to.z - from.z;
    len = sqrt(pow(x, 2) + pow(y, 2));
}

CalcVector::CalcVector(const geometry_msgs::Pose &in_pose)
{
    x = cos(Ras::quatToYaw(in_pose.orientation));
    y = sin(Ras::quatToYaw(in_pose.orientation));
    len = 1.0;
}
