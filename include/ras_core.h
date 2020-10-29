#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <derived_object_msgs/ObjectArray.h>
#include <derived_object_msgs/Object.h>
#include <nav_msgs/Odometry.h>
#include <shape_msgs/SolidPrimitive.h>
#include <std_msgs/Int8.h>

#include "ras_lib.h"
#include "ras/RasObject.h"
#include "ras/RasObjectArray.h"
#include <ras/rasConfig.h>

// #include "carla_msgs/CarlaActorList.h"
#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/Waypoint.h"

#include <dynamic_reconfigure/server.h>
#include <unordered_map>
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
    ros::Publisher pub_wp_cross_twist;
    ros::Publisher pub_wp_cross_pose;
    ros::Publisher pub_wp_obj;
    ros::Publisher pub_intervene_type;

    int m_keep_time;
    float m_max_vision;
    float m_min_vision;
    std::string m_ego_name;
	int m_intervene_type;
    bool m_conservative_recognition;
    float m_detection_level_thres;

    std::unordered_map<int, ras::RasObject> m_obj_map;
    std::map<int, std::vector<int>> m_wp_obj_map;
    geometry_msgs::Pose m_ego_pose;
    geometry_msgs::Twist m_ego_twist;
    int m_ego_id;
    std::vector<geometry_msgs::Pose> m_waypoints;
    int m_ego_wp;
    int m_brakable_wp;
    float m_wp_interval;

    dynamic_reconfigure::Server<ras::rasConfig> server;
    dynamic_reconfigure::Server<ras::rasConfig>::CallbackType server_callback;

public:
	RasCore();

private:
    void callbackDynamicReconfigure(ras::rasConfig &config, uint32_t lebel);
    void subTrajectoryCallback(const autoware_msgs::LaneArray &in_array);
	// void subActorCallback(const carla_msgs::CarlaActorList &in_actor_list);
    void subOdomCallback(const nav_msgs::Odometry &in_odom);
    void subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array);
    std::vector<int> findWpOfObj(ras::RasObject &obj);
    void manageMarkers();
    void subShiftCallback(const ras::RasObject &in_msg);
    void addObjIdToOccupancyWp(const std::vector<int> &in_waypoint, const ras::RasObject &in_obj);
    bool isCollideObstacle(const ras::RasObject &in_obj, const int &wp);
    int findWallWp(std::vector<int> &critical_obj_id_vec);
    void pubOccupancyWp(const geometry_msgs::Point &in_pose, const int &type);
    bool isSameDirection(const RasVector &vec_1, const RasVector &vec_2, const float &thres);
    bool isPerpendicular(const RasVector &vec_1, const RasVector &vec_2, const float &thres);
};
