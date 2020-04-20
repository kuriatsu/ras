#include <ros/ros.h>
#include "ras_visualizer.h"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

RasVisualizer::RasVisualizer(): marker_scale(1.0), marker_vertical_shrink_rate(0.1)
{
	ros::NodeHandle n;
    server.reset(new interactive_markers::InteractiveMarkerServer("ras_visualizer_node"));

    sub_obj = n.subscribe("/managed_objects", 5, &RasVisualizer::subObjCallback, this);
	// sub_vehicle_info = n.subscribe("/ego_vehicle/", 5, &RasVisualizer::subVehicleInfoCallback, this);
	// sub_erase_signal = n.subscribe("/erase_signal", 5, &RasVisualizer::erase_signal_callback, this);
	pub_shift = n.advertise<ras::RasObject>("/shifted_info", 5);
}


RasVisualizer::~RasVisualizer()
{
    server.reset();
}

void RasVisualizer::subObjCallback(const ras::RasObjectArray &in_obj_array)
{
    server->clear();
    // ROS_INFO("visualezer subscribed");

    for (size_t i=0; i< in_obj_array.objects.size(); i++)
    {
        createInteractiveMarker(in_obj_array.objects[i]);
    }

    server->applyChanges();
}


void RasVisualizer::createInteractiveMarker(ras::RasObject in_obj)
{
    // for debag
    // std::cout <<"ss id is:" << in_obj.id << std::endl;
    std::stringstream ss;
    ss << in_obj.object.id;

	visualization_msgs::InteractiveMarker int_marker;
    // ROS_INFO_STREAM(in_obj);
	int_marker.header.frame_id = in_obj.object.header.frame_id;
	int_marker.name = ss.str();
	int_marker.scale = marker_scale;
    int_marker.pose = in_obj.object.pose;
    int_marker.pose.position.x = in_obj.object.pose.position.x + in_obj.shift_x;
    int_marker.pose.position.y = in_obj.object.pose.position.y + in_obj.shift_y;
    if(in_obj.object.classification == 4)
    {
        int_marker.pose.position.z = in_obj.object.pose.position.z - in_obj.object.shape.dimensions[2] * (1 - marker_vertical_shrink_rate) * 0.5;
    }
    else if (in_obj.object.classification == 6)
    {
        int_marker.pose.position.z = in_obj.object.pose.position.z;
    }

    setMarkerControl(int_marker, in_obj);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&RasVisualizer::shiftFeedback, this, _1));
}


void RasVisualizer::setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, ras::RasObject in_obj)
{
	visualization_msgs::InteractiveMarkerControl control;

	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.orientation.w = 1;

    setMarkerToMarkerControl(control, in_obj);
	// control.markers.push_back(setMarkerToMarkerControl(in_obj));
    int_marker.controls.push_back(control);
}


void RasVisualizer::setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, ras::RasObject in_obj)
{
    visualization_msgs::Marker marker;

    marker.ns = "ras";
    marker.id = in_obj.object.id;

    if (in_obj.object.classification == 4)
    {
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.scale.x = marker_scale*in_obj.object.shape.dimensions[0];
        marker.scale.y = marker_scale*in_obj.object.shape.dimensions[1];
        marker.scale.z = marker_scale*in_obj.object.shape.dimensions[2] * marker_vertical_shrink_rate;
    }

    else if (in_obj.object.classification == 5)
    {
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = marker_scale*in_obj.object.shape.dimensions[0];
        marker.scale.y = marker_scale*in_obj.object.shape.dimensions[1];
        marker.scale.z = marker_scale*in_obj.object.shape.dimensions[2] * marker_vertical_shrink_rate;
    }

    else if (in_obj.object.classification == 6)
    {
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = marker_scale*in_obj.object.shape.dimensions[0];
        marker.scale.y = marker_scale*in_obj.object.shape.dimensions[1];
        marker.scale.z = marker_scale*in_obj.object.shape.dimensions[2] * marker_vertical_shrink_rate;
    }

    if(in_obj.importance == 1.0)
    {
        marker.color.r = 1;
        marker.color.g = 0;
        marker.color.b = 0;
    }
    else if(in_obj.importance == 0.5)
    {
        marker.color.r = 0;
        marker.color.g = 1;
        marker.color.b = 0;
    }
    else
    {
        marker.color.r = 0;
        marker.color.g = 0;
        marker.color.b = 1;
    }
    marker.color.a = 0.3;

    // marker.lifetime = ros::Duration(2.0);
    control.markers.push_back(marker);
}


void RasVisualizer::shiftFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ras::RasObject feedback_obj;
    std::istringstream sis;

    sis = std::istringstream(feedback->marker_name);
    feedback_obj.object.pose = feedback->pose;
    sis >> feedback_obj.object.id;
    pub_shift.publish(feedback_obj);
}
