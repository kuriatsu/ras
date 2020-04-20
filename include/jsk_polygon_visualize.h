#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_rviz_plugins/PictogramArray.h>
#include <cmath>

#include "ras/RasObject.h"
#include "ras/RasObjectArray.h"
#include "ras_lib.h"
#include "ras_visualizer.h"
// #include "std_msgs/Int32.h"


class RasVisualizer
{
private:

    ros::Subscriber sub_obj;
    ros::Subscriber sub_wall;
    ros::Publisher pub_fb_obj;
    ros::Publisher pub_polygon;
    ros::Publisher pub_wall;
    ros::Publisher pub_pictgram;
    std::vector<uint32_t> id_vec;
    float marker_scale;
    // float marker_vertical_shrink_rate;
public:
    RasVisualizer();
	~RasVisualizer();
	void sync_jsk_box();

private:
    void subObjCallback(const ras::RasObjectArray &in_obj_array);
    void subWallCallback(const ras::RasObject &in_obj);
    void intMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    void createPolygon(const ras::RasObject &in_obj, jsk_recognition_msgs::PolygonArray &polygon_array);
    void createInteractiveMarker(ras::RasObject &in_obj);
    void setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const ras::RasObject &in_obj);
    void setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const ras::RasObject &in_obj);
    jsk_rviz_plugins::Pictogram createPictogram(const ras::RasObject &in_obj, const int &type);
};
