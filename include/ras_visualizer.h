#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sound_play/sound_play.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_rviz_plugins/PictogramArray.h>
// #include <jsk_recognition_msgs/PolygonArray.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>

#include "ras/RasObject.h"
#include "ras/RasObjectArray.h"
#include "ras_lib.h"
#include "ras_visualizer.h"
// #include "std_msgs/Int32.h"

#include <cmath>

class RasVisualizer
{
private:

    ros::Subscriber sub_obj;
    ros::Subscriber sub_wall;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_key_input;
    ros::Subscriber sub_joy_input;
    // ros::Subscriber sub_intervene_type;
    ros::Publisher pub_fb_obj;
    ros::Publisher pub_box;
    ros::Publisher pub_wall;
    ros::Publisher pub_pictgram;
    ros::Publisher pub_camera_angle;

    std::vector<uint32_t> id_vec;
    float marker_scale;
    jsk_rviz_plugins::PictogramArray m_pictogram_list;
    // std::vector<jsk_rviz_plugins::Pictogram> wall_pictogram;
    // std::vector<visualization_msgs::Marker> wall_marker;
    std::vector<int> m_important_objects;
    std::vector<int> m_last_intervened_objects;
    std::vector<int> m_beeped_object_history;
    ros::Time m_last_touch_time;
    std::string m_ego_name = "ego_vehicle";
    // ros::Time last_wall_time;
    // int intervene_type; // 0:control 1:enter 2:touch
    sound_play::SoundClient sound_client;

    float m_vehicle_deceleration; // m/s^2
    geometry_msgs::Twist m_ego_twist;
    float m_beep_timing;
    float m_deceleration_start_distance;
    float m_stop_distance_to_object;

public:
    RasVisualizer();
	~RasVisualizer();
	void sync_jsk_box();

private:
    void subObjCallback(const ras::RasObjectArray &in_obj_array);
    void subOdomCallback(const nav_msgs::Odometry &in_odom);
    void subWallCallback(const ras::RasObject &in_obj);
    void intMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
    jsk_recognition_msgs::BoundingBox createBox(const ras::RasObject &in_obj);
    visualization_msgs::Marker createMarker(const ras::RasObject &in_obj);
    void createInteractiveMarker(ras::RasObject &in_obj);
    void setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const ras::RasObject &in_obj);
    void setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const ras::RasObject &in_obj);
    jsk_rviz_plugins::Pictogram createPictogram(const ras::RasObject &in_obj, const int &type);
    void subButtonInputCallback(const std_msgs::String &in_key);
    void subJoyInputCallback(const sensor_msgs::Joy &in_joy);
    // void subInterveneTypeCallback(const std_msgs::Int32 &intervene_type);
};
