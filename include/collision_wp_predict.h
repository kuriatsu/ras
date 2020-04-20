#pragma once

#include <ros/ros.h>
#include <derived_object_msgs/Object.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include "ras_lib.h"
#include "ras/RasObject.h"
#include "ras/RasObjectArray.h"
#include <ras/rasConfig.h>

#include "autoware_msgs/LaneArray.h"
#include "autoware_msgs/Waypoint.h"

#include <dynamic_reconfigure/server.h>
#include <unordered_map>
#include <math.h>

class CollisionWpPredict
{
private:
	ros::Publisher pub_wall;
	ros::Publisher pub_wp_cross;
	ros::Publisher pub_wp_obj;
	ros::Subscriber sub_trajectory;
	ros::Subscriber sub_obj;
	ros::Subscriber sub_odom;

	std::map<int, std::vector<int>> m_wp_obj_map;
	std::vector<geometry_msgs::Pose> m_wps_vec;
	geometry_msgs::Pose m_ego_pose;
	geometry_msgs::Twist m_ego_twist;
	int m_ego_wp;
	int m_brakable_wp;
	float m_wp_interval;
	float m_max_vision;
    float m_min_vision;
	
	dynamic_reconfigure::Server<ras::rasConfig> server;
    dynamic_reconfigure::Server<ras::rasConfig>::CallbackType server_callback;

public:
	CollisionWpPredict();

private:
	void callbackDynamicReconfigure(ras::rasConfig &config, uint32_t lebel);
	void subTrajectoryCallback(const autoware_msgs::LaneArray &in_array);
	void subOdomCallback(const nav_msgs::Odometry &in_odom);
	void subObjCallback(const ras::RasObjectArray &in_obj_array);
	std::vector<int> findWpOfObj(ras::RasObject &in_obj);
	void calcOccupancyWp(const std::vector<int> &in_wp_vec, const ras::RasObject &in_obj);
	void findWallWp(int &wall_wp_id, std::vector<int> &critical_obj_id_vec);
	void pubOccupancyWp(const geometry_msgs::Point &in_pose, const int &type);
	bool isCollideObstacle(const ras::RasObject &in_obj, const int &wp);

}
