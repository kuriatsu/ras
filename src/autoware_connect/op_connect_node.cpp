#include <ros/ros.h>
#include "ras_autoware_connector.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ras_autoware_connector_node");
    ROS_INFO("Initializing...");
    RasAutowareConnector ras_autoware_connector;
    ROS_INFO("Ready...");
    ros::spin();

    return 0;
}
