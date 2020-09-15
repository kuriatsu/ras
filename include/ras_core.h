#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <nav_msgs/Odometry.h>
#include <shape_msgs/SolidPrimitive.h>

#include "ras_lib.h"
#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"
#include <ras_carla/rasConfig.h>

#include "carla_msgs/CarlaActorList.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/Waypoint.h"

#include <dynamic_reconfigure/server.h>
#include <unordered_map>
#include <math.h>
#include <typeinfo>

class RasCore
{
private:
    ros::Publisher pub_obj;
    ros::Publisher pub_wall;
	ros::Subscriber sub_carla_actor_list;
	ros::Subscriber sub_carla_obj;
    ros::Subscriber sub_shift;
	ros::Subscriber sub_odom;
    ros::Subscriber sub_trajectory;
    ros::Publisher pub_wp_cross;
    ros::Publisher pub_wp_obj;

    int m_keep_time;
    float m_max_vision;
    float m_min_vision;
	std::string m_ego_name;
    bool m_conservative_recognition;

    std::unordered_map<int, ras_carla::RasObject> m_obj_map;
    std::map<int, std::vector<int>> m_wp_obj_map;
    geometry_msgs::Pose m_ego_pose;
    geometry_msgs::Twist m_ego_twist;
    int m_ego_id;
    std::vector<geometry_msgs::Pose> m_wps_vec;
    int m_ego_wp;
    int m_brakable_wp;
    float m_wp_interval;

    dynamic_reconfigure::Server<ras_carla::rasConfig> server;
    dynamic_reconfigure::Server<ras_carla::rasConfig>::CallbackType server_callback;

public:
	RasCore();

private:
    void callbackDynamicReconfigure(ras_carla::rasConfig &config, uint32_t lebel);
    void subTrajectoryCallback(const autoware_msgs::LaneArray &in_array);
	// void subActorCallback(const carla_msgs::CarlaActorList &in_actor_list);
    void subOdomCallback(const nav_msgs::Odometry &in_odom);
    void subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array);
    std::vector<int> findWpOfObj(ras_carla::RasObject &obj);
    void manageMarkers();
    void subShiftCallback(const ras_carla::RasObject &in_msg);
    void calcOccupancyWp(const std::vector<int> &in_wp_vec, const ras_carla::RasObject &in_obj);
    bool isCollideObstacle(const ras_carla::RasObject &in_obj, const int &wp);
    int findWallWp(std::vector<int> &critical_obj_id_vec);
    void pubOccupancyWp(const geometry_msgs::Point &in_pose, const int &type);
    bool isSameDirection(const RasVector &vec_1, const RasVector &vec_2, const float &thres);
    bool isPerpendicular(const RasVector &vec_1, const RasVector &vec_2, const float &thres);
};
