#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
// #include <jsk_recognition_msgs/PolygonArray.h>
#include <visualization_msgs/Marker.h>
#include <jsk_rviz_plugins/PictogramArray.h>
#include <cmath>

#include "ras_carla/RasObject.h"
#include "ras_carla/RasObjectArray.h"
#include "ras_lib.h"
#include "ras_visualizer.h"
// #include "std_msgs/Int32.h"


class RasVisualizer
{
private:

    ros::Subscriber sub_obj;
    ros::Subscriber sub_wall;
    ros::Publisher pub_fb_obj;
    ros::Publisher pub_box;
    ros::Publisher pub_wall;
    ros::Publisher pub_pictgram;

    std::vector<uint32_t> id_vec;
    float marker_scale;
    std::vector<jsk_rviz_plugins::Pictogram> wall_pictogram;
    std::vector<visualization_msgs::Marker> wall_marker;

public:
    RasVisualizer();
	~RasVisualizer();
	void sync_jsk_box();

private:
    void subObjCallback(const ras_carla::RasObjectArray &in_obj_array);
    void subWallCallback(const ras_carla::RasObject &in_obj);
    void intMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    jsk_recognition_msgs::BoundingBox createBox(const ras_carla::RasObject &in_obj);
    visualization_msgs::Marker createMarker(const ras_carla::RasObject &in_obj);
    void createInteractiveMarker(ras_carla::RasObject &in_obj);
    void setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const ras_carla::RasObject &in_obj);
    void setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const ras_carla::RasObject &in_obj);
    jsk_rviz_plugins::Pictogram createPictogram(const ras_carla::RasObject &in_obj, const int &type);
};
