#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_server.h>
#include <cmath>

#include "ras/RasObject.h"
#include "ras/RasObjectArray.h"
#include "ras_visualizer.h"
// #include "std_msgs/Int32.h"


class RasVisualizer
{
private:

        ros::Subscriber sub_obj;
        // ros::Subscriber sub_erase_signal;
        // ros::Subscriber sub_vehicle_info;
        ros::Publisher pub_shift;
        std::vector<uint32_t> id_vec;
        float marker_scale;
        float marker_vertical_shrink_rate;
public:
    RasVisualizer();
	~RasVisualizer();
	void sync_jsk_box();

private:
        void subObjCallback(const ras::RasObjectArray &in_obj_array);
        void shiftFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
        void createInteractiveMarker(ras::RasObject in_obj);
        void setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, ras::RasObject in_obj);
        void setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, ras::RasObject in_obj);
};
