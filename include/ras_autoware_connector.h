#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>

#include <autoware_msgs/DetectedObject.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include "ras_carla/RasObjectArray.h"
#include "ras_carla/RasObject.h"

#include "ras_lib.h"

class RasAutowareConnector{

private:
    ros::Subscriber sub_obj;
    ros::Subscriber sub_wall;
    ros::Publisher pub_obj;
    // ros::Publisher pub_polygon;
    float polygon_interval;
    std::vector<autoware_msgs::DetectedObject> wall;
public :
    RasAutowareConnector();

private :
    void subWallCallback(const ras_carla::RasObject &in_obj);
    void subObjCallback(const ras_carla::RasObjectArray &in_obj_array);
    autoware_msgs::DetectedObject rasToAutowareObject(const ras_carla::RasObject &in_obj);
    geometry_msgs::PolygonStamped calcPolygon(const ras_carla::RasObject &in_obj);
};
