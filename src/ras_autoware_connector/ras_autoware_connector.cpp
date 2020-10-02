#include <ros/ros.h>
#include "ras_autoware_connector.h"

RasAutowareConnector::RasAutowareConnector(): polygon_interval(0.5)//, keep_time(2)
{
    ros::NodeHandle n;

    sub_obj = n.subscribe("/managed_objects", 5, &RasAutowareConnector::subObjCallback, this);
    sub_wall = n.subscribe("/wall_object", 5, &RasAutowareConnector::subWallCallback, this);
    pub_obj = n.advertise<autoware_msgs::DetectedObjectArray>("/tracked_objects", 5);
    // pub_polygon = n.advertise<geometry_msgs::PolygonStamped>("/ras_polygon", 10);
}


void RasAutowareConnector::subWallCallback(const ras::RasObject &in_obj)
{
    wall.emplace_back(rasToAutowareObject(in_obj));
}


void RasAutowareConnector::subObjCallback(const ras::RasObjectArray &in_obj_array)
{
    autoware_msgs::DetectedObjectArray out_obj_array;
    autoware_msgs::DetectedObject out_obj;
    out_obj_array.header = in_obj_array.header;

    for (size_t index = 0; index < in_obj_array.objects.size(); index++)
    {
        if (in_obj_array.objects[index].object.classification == derived_object_msgs::Object::CLASSIFICATION_UNKNOWN && in_obj_array.objects[index].is_touched)
        {
            continue;
        }

        out_obj = rasToAutowareObject(in_obj_array.objects[index]);
        out_obj_array.objects.emplace_back(out_obj);
    }
    // std::cout << "obj stamp : " <<out_obj_array.header.stamp << " wall stamp : "  <<  wall.header.stamp<< std::endl;

    if (!wall.empty())
    {
        out_obj_array.objects.emplace_back(wall[0]);
        std::cout << "added wall" << std::endl;
        wall.pop_back();
    }

    pub_obj.publish(out_obj_array);
}


autoware_msgs::DetectedObject RasAutowareConnector::rasToAutowareObject(const ras::RasObject &in_obj)
{
    autoware_msgs::DetectedObject out_obj;

    out_obj.header = in_obj.object.header;
    out_obj.id = in_obj.object.id;
    // out_obj.label = in_obj.object.label;
    out_obj.score = in_obj.object.classification_certainty;
    // out_obj.color = color;
    out_obj.valid = false;
    out_obj.space_frame = "";
    out_obj.pose = in_obj.object.pose;
    out_obj.dimensions.x = in_obj.object.shape.dimensions[0];
    out_obj.dimensions.y = in_obj.object.shape.dimensions[1];
    out_obj.dimensions.z = in_obj.object.shape.dimensions[2];
    // out_obj.valiance = {0.0, 0.0, 0.0};
    out_obj.velocity = in_obj.object.twist;
    out_obj.acceleration.linear.x = in_obj.object.accel.linear.x;
    out_obj.acceleration.linear.y = in_obj.object.accel.linear.y;
    out_obj.acceleration.linear.x = in_obj.object.accel.linear.z;
    out_obj.acceleration.angular.x = in_obj.object.accel.angular.x;
    out_obj.acceleration.angular.y = in_obj.object.accel.angular.y;
    out_obj.acceleration.angular.z = in_obj.object.accel.angular.z;
    // out_obj.pointcloud = in_obj.object.pose;
    out_obj.convex_hull = calcPolygon(in_obj);
    // pub_polygon.publish(out_obj.convex_hull);
    out_obj.pose.position.x = in_obj.object.pose.position.x;
    out_obj.pose.position.y = in_obj.object.pose.position.y;
    // out_obj.candidate_trajectories = in_obj.object.pose;
    out_obj.pose_reliable = true;
    out_obj.velocity_reliable = true;
    out_obj.acceleration_reliable = false;
    // out_obj.image_frame = true;
    out_obj.x = 0;
    out_obj.y = 0;
    out_obj.width = 0;
    out_obj.height = 0;
    out_obj.angle = 0.0;
    // out_obj.roi_image = true;
    out_obj.indicator_state = 0;
    out_obj.behavior_state = 0;

    return out_obj;
}


geometry_msgs::PolygonStamped RasAutowareConnector::calcPolygon(const ras::RasObject &in_obj)
{
    geometry_msgs::Point32 point;
    geometry_msgs::PolygonStamped polygon_stamped;
    float x, y;

    polygon_stamped.header = in_obj.object.header;

    int polygon_num_x = in_obj.object.shape.dimensions[0] / polygon_interval;
    int polygon_num_y = in_obj.object.shape.dimensions[1] / polygon_interval;
    double yaw = Ras::quatToYaw(in_obj.object.pose.orientation);

    for (int i = 0; i < polygon_num_x; i++)
    {
        x = ( -in_obj.object.shape.dimensions[0] / 2 + polygon_interval * i );
        y = ( in_obj.object.shape.dimensions[1] / 2 );
        point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x;
        point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y;
        point.z = 0.0;
        polygon_stamped.polygon.points.emplace_back(point);

        y = ( -in_obj.object.shape.dimensions[1] / 2 );
        point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x;
        point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y;
        polygon_stamped.polygon.points.emplace_back(point);
    }

    for (int i = 0; i < polygon_num_y; i++)
    {
        x = ( in_obj.object.shape.dimensions[0] / 2 );
        y = ( -in_obj.object.shape.dimensions[1] / 2 + polygon_interval * i );
        point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x;
        point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y;
        point.z = 0.0;
        polygon_stamped.polygon.points.emplace_back(point);

        x = ( -in_obj.object.shape.dimensions[0] / 2 );
        point.x = x * cos( 2 * M_PI + yaw ) - y * sin( 2 * M_PI + yaw ) + in_obj.object.pose.position.x;
        point.y = x * sin( 2 * M_PI + yaw ) + y * cos( 2 * M_PI + yaw ) + in_obj.object.pose.position.y;
        polygon_stamped.polygon.points.emplace_back(point);
    }
    return polygon_stamped;
}
