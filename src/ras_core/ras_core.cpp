#include <ros/ros.h>
#include "ras_core.h"

RasCore::RasCore(): m_ego_wp(0)
{
    ros::NodeHandle n;

    server_callback = boost::bind(&RasCore::callbackDynamicReconfigure, this, _1, _2);
    server.setCallback(server_callback);

    sub_trajectory = n.subscribe("/lane_waypoints_array", 5, &RasCore::subTrajectoryCallback, this);
    // sub_carla_actor_list = n.subscribe("/carla/actor_list", 1, &RasCore::subActorCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &RasCore::subOdomCallback, this);
    sub_carla_obj = n.subscribe("/my_carla_actors", 1, &RasCore::subObjCallback, this);
    sub_shift = n.subscribe("/feedback_object", 10, &RasCore::subShiftCallback, this);

    pub_obj = n.advertise<ras::RasObjectArray>("/managed_objects", 5);
    pub_wall = n.advertise<ras::RasObject>("/wall_object", 1);
    pub_wp_obj = n.advertise<geometry_msgs::PointStamped>("/obj_wp", 5);
    pub_wp_cross_twist = n.advertise<geometry_msgs::PointStamped>("/cross_twist_wp", 5);
    pub_wp_cross_pose = n.advertise<geometry_msgs::PointStamped>("/cross_pose_wp", 5);

    // pub_erase = n.advertise<std_msgs::Int32>("/erase_signal", 1);


    m_obj_map.clear();
}


void RasCore::callbackDynamicReconfigure(ras::rasConfig &config, uint32_t lebel)
{
    m_conservative_recognition = config.conservative?true:false;
    m_max_vision = config.max_vision_range;
    m_min_vision = config.min_vision_range;
    m_keep_time = config.keep_time;
    m_ego_name = config.ego_name;
    m_detection_level_thres = config.detection_level_thres;
}

// get ego_vehicle id to remove ego_object from obstacles
// void RasCore::subActorCallback(const carla_msgs::CarlaActorList &in_actor_list)
// {
//     // ROS_INFO("subActorCallback");
// 	for (size_t i = 0; i < in_actor_list.actors.size(); i ++)
// 	{
// 		if (in_actor_list.actors[i].rolename == m_ego_name)
// 		{
// 			m_ego_id = in_actor_list.actors[i].id;
// 		}
// 	}
// }

void RasCore::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
    // ROS_INFO("subOdomCallback");
    float min_dist_power = 9.0, dist;
    int ego_wp = 0;
    m_ego_pose = in_odom.pose.pose;
    m_ego_twist = in_odom.twist.twist;
    // std::cout << (int)(100.0 / m_wp_interval) << std::endl;
    // find closest waypoint from m_waypoints
    // for (size_t i = m_ego_wp; i < m_ego_wp + (int)(m_max_vision / m_wp_interval); i++)
    for (size_t i = 0; i < m_waypoints.size(); i++)
    {
        dist = pow(m_ego_pose.position.x - m_waypoints[i].position.x, 2) + pow(m_ego_pose.position.y - m_waypoints[i].position.y, 2);
        // std::cout << i << "," << dist << std::endl;
        if (min_dist_power > dist)
        {
            ego_wp = i;
            min_dist_power = dist;
        }
    }

    m_ego_wp = ego_wp;
    m_brakable_wp = ego_wp + (int)((pow(m_ego_twist.linear.x, 2) / (2 * 9.8 * 0.2)) / m_wp_interval);
    // m_brakable_wp = ego_wp + (int)((pow(m_ego_twist.linear.x, 2) / (2 * 9.8 * 0.3)));
    // m_brakable_wp = ego_wp + (int)((pow(m_ego_twist.linear.x * 3.6, 2) / (254 * 0.7)) / m_wp_interval);

}


void RasCore::subTrajectoryCallback(const autoware_msgs::LaneArray &in_array)
{
    // ROS_INFO("subTrajectoryCallback");
    if (in_array.lanes[0].waypoints.empty())
    {
        ROS_ERROR("waypoint_publisher: waypoints empty");
        return;
    }
    for (const auto &itr : in_array.lanes[0].waypoints)
    {
        m_waypoints.emplace_back(itr.pose.pose);
    }
    m_wp_interval = sqrt(pow(m_waypoints[0].position.x - m_waypoints[1].position.x, 2) + pow(m_waypoints[0].position.y - m_waypoints[1].position.y, 2));
}


void RasCore::subObjCallback(const derived_object_msgs::ObjectArray &in_obj_array)
{
    // ROS_INFO("subObjCallback");
    if (m_waypoints.empty())
    {
        ROS_ERROR("waypoint_publisher: waypoint or ego odometry is not subscrived yet");
        return;
    }
    else if(m_ego_wp == 0)
    {
        ROS_ERROR("ego_vehicle is far from over 9.0 m from route");
        return;
    }

    m_wp_obj_map.clear();

    geometry_msgs::Pose in_obj_pose;
    ras::RasObject ras_obj;
    std::vector<int> wp_vec;

    for (const auto &e : in_obj_array.objects)
    {
        if (e.pose.position.x == 0.0 && e.pose.position.y == 0.0 && e.pose.position.z == 0.0) continue;

        ras_obj.object = e;
        in_obj_pose = Ras::tfTransformer(ras_obj.object.pose, ras_obj.object.header.frame_id, m_ego_name);
        ras_obj.distance = sqrt(pow(in_obj_pose.position.x, 2) + pow(in_obj_pose.position.y, 2));
        if (m_min_vision < ras_obj.distance && ras_obj.distance < m_max_vision)
        {
            ras::RasObject &selected_obj = m_obj_map[ras_obj.object.id];
            selected_obj.object = ras_obj.object;
            selected_obj.distance = ras_obj.distance;
            selected_obj.is_interaction = false;
            selected_obj.is_important = false;
            addObjIdToOccupancyWp(findWpOfObj(selected_obj), selected_obj);

        }
    }
    manageMarkers();
}


/*
get waypoint related to in_obj
check distance, the same direction or perpendicular from object
*/
std::vector<int> RasCore::findWpOfObj(ras::RasObject &in_obj)
{
    std::vector<int> obj_waypoints;

    switch(in_obj.object.classification)
    {
        case derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN:
        {
            // ROS_INFO("findWpOfObj pedestrian");
            float min_dist_of_wp_obj = m_max_vision, dist_of_wp_obj;
            int close_wp = 0, twist_cross_wp = 0, pose_cross_wp = 0;
            RasVector obj_twist_vec(in_obj.object.twist);
            RasVector obj_pose_vec(in_obj.object.pose);

            for (auto itr = m_waypoints.begin(); itr < m_waypoints.end(); itr++)
            {


                // find crossing waypoint with object twist
                RasVector obj_wp_vec(in_obj.object.pose.position, itr->position);

                if (isSameDirection(obj_wp_vec, obj_twist_vec, 0.9999))
                {
                    // std::cout << "obj_id : " << in_obj.object.id << " wp : " << close_wp << "close" << std::endl;
                    twist_cross_wp = std::distance(m_waypoints.begin(), itr);
                    obj_waypoints.emplace_back(twist_cross_wp);
                    pubOccupancyWp(m_waypoints[twist_cross_wp].position, 1);
                }

                dist_of_wp_obj = Ras::calcDistOfPoints(itr->position, in_obj.object.pose.position);


                if (in_obj.object.detection_level < m_detection_level_thres) continue;

                // find closest waypoint from object
                if (dist_of_wp_obj < min_dist_of_wp_obj)
                {
                    min_dist_of_wp_obj = dist_of_wp_obj;
                    close_wp = std::distance(m_waypoints.begin(), itr);
                }

                // find crossing waypoint with object pose
                if (isSameDirection(obj_wp_vec, obj_pose_vec, 0.9999))
                {
                    pose_cross_wp = std::distance(m_waypoints.begin(), itr);
                    obj_waypoints.emplace_back(pose_cross_wp);
                    pubOccupancyWp(m_waypoints[pose_cross_wp].position, 2);
                }
            }

            if (close_wp != 0 && in_obj.object.detection_level < m_detection_level_thres)
            {
                dist_of_wp_obj = Ras::calcDistOfPoints(m_waypoints[close_wp].position, in_obj.object.pose.position);

                if (dist_of_wp_obj > 1.0)
                {
                    obj_waypoints.emplace_back(close_wp);
                    pubOccupancyWp(m_waypoints[close_wp].position, 0);
                }
            }



            // RasVector obj_closewp_vec(in_obj.object.pose.position, m_waypoints[close_wp].position);
            // RasVector obj_vec(in_obj.object.pose);
            //
            // if (isSameDirection(obj_closewp_vec, obj_vec, 0.5))
            // {
            //     // std::cout << "obj_id : " << in_obj.object.id << " wp : " << close_wp << "close" << std::endl;
            //     obj_waypoints.emplace_back(close_wp);
            //     pubOccupancyWp(m_waypoints[close_wp].position, 0);
            // }

            // find perpendicular waypoint from close waypoint and object
            // for (auto itr = m_waypoints.begin(); itr != m_waypoints.end(); itr++)
            // {
            //     perp_wp = std::distance(m_waypoints.begin(), itr);
            //
            //     if(perp_wp == 0) continue;
            //     if(perp_wp == close_wp) continue;
            //
            //     RasVector obj_perpwp_vec(in_obj.object.pose.position, itr->position);
            //
            //     if (isPerpendicular(obj_closewp_vec, obj_perpwp_vec, 0.05) || isSameDirection(obj_closewp_vec, obj_perpwp_vec, 0.8))
            //     {
            //         // is the path and obj-wp perpendicular?
            //         RasVector path_vec((itr-1)->position, itr->position);
            //
            //         if (isPerpendicular(path_vec, obj_perpwp_vec, 0.05) && isSameDirection(obj_vec, obj_perpwp_vec, 0.7))
            //         {
            //             // std::cout << "obj_id : " << in_obj.object.id << " wp : " << perp_wp << "cross" << std::endl;
            //             obj_waypoints.emplace_back(perp_wp);
            //             pubOccupancyWp(m_waypoints[perp_wp].position, 1);
            //         }
            //     }
            // }
            break;
        }

        case derived_object_msgs::Object::CLASSIFICATION_CAR:
        {
            float dist_of_wp_ego;
            int pose_cross_wp;
            RasVector obj_twist_vec(in_obj.object.twist);
            RasVector obj_pose_vec(in_obj.object.pose);

            // std::cout << obj_twist_vec.x << ", " << obj_twist_vec.y << ", " << obj_twist_vec.len << std::endl;
            for (auto itr = m_waypoints.begin(); itr != m_waypoints.end(); itr ++)
            {
                RasVector obj_wp_vec(in_obj.object.pose.position, itr->position);


                if (isSameDirection(obj_twist_vec, obj_wp_vec, 0.9999))
                {
                    if (Ras::calcDistOfPoints(itr->position, in_obj.object.pose.position) < in_obj.object.shape.dimensions[1]) break;
                    pubOccupancyWp(m_waypoints[std::distance(m_waypoints.begin(), itr)].position, 1);
                    obj_waypoints.emplace_back(std::distance(m_waypoints.begin(), itr));
                    // break;
                }

                if (in_obj.object.detection_level < m_detection_level_thres) continue;

                // find crossing waypoint with object pose
                if (isSameDirection(obj_wp_vec, obj_pose_vec, 0.9999))
                {
                    if (Ras::calcDistOfPoints(itr->position, in_obj.object.pose.position) < in_obj.object.shape.dimensions[1]) break;
                    pose_cross_wp = std::distance(m_waypoints.begin(), itr);
                    obj_waypoints.emplace_back(pose_cross_wp);
                    pubOccupancyWp(m_waypoints[pose_cross_wp].position, 2);
                }

            }
            break;
        }

        case derived_object_msgs::Object::CLASSIFICATION_UNKNOWN:
        {
            float min_dist_of_wp_obj = m_max_vision, dist_of_wp_obj;
            int close_wp;
            // find closest waypoint from object
            for (auto itr = m_waypoints.begin(); itr < m_waypoints.end(); itr++)
            // for (auto itr = m_waypoints.begin(); itr != m_waypoints.end(); itr++)
            {
                dist_of_wp_obj = Ras::calcDistOfPoints(itr->position, in_obj.object.pose.position);
                if (dist_of_wp_obj < min_dist_of_wp_obj)
                {
                    min_dist_of_wp_obj = dist_of_wp_obj;
                    close_wp = std::distance(m_waypoints.begin(), itr);
                }
            }
            obj_waypoints.emplace_back(close_wp);
            pubOccupancyWp(m_waypoints[close_wp].position, 0);
        }
    }
    return obj_waypoints;
}


bool RasCore::isSameDirection(const RasVector &vec_1, const RasVector &vec_2, const float &thres)
{
    // is close wp same direction with the object
    float inner_prod = vec_1.x * vec_2.x + vec_1.y * vec_2.y;
    return (inner_prod > vec_1.len * vec_2.len * thres);
}


bool RasCore::isPerpendicular(const RasVector &vec_1, const RasVector &vec_2, const float &thres)
{
    float inner_prod = vec_1.x * vec_2.x + vec_1.y * vec_2.y;
    return (fabs(inner_prod) < vec_1.len * vec_2.len * thres);
}


/*
check the collition between in_obj and ego_vehicle
*/
bool RasCore::isCollideObstacle(const ras::RasObject &in_obj, const int &wp)
{
    float dist_of_wp_ego, dist_of_wp_obj;
    dist_of_wp_ego = (wp - m_ego_wp) * m_wp_interval;
    dist_of_wp_obj = Ras::calcDistOfPoints(in_obj.object.pose.position, m_waypoints[wp].position);

    if (wp > m_ego_wp + m_max_vision / m_wp_interval || in_obj.is_touched || dist_of_wp_ego < 0)
        return false;

    switch (in_obj.object.classification)
    {
        case derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN:
        {
            std::cout << wp << "; " << dist_of_wp_obj << " " << dist_of_wp_ego << " " << dist_of_wp_obj / sqrt(pow(in_obj.object.twist.linear.x, 2) + pow(in_obj.object.twist.linear.y, 2)) << " " << dist_of_wp_ego / m_ego_twist.linear.x << std::endl;
            return (dist_of_wp_obj < dist_of_wp_ego || (dist_of_wp_obj / sqrt(pow(in_obj.object.twist.linear.x, 2) + pow(in_obj.object.twist.linear.y, 2))) < (dist_of_wp_ego / m_ego_twist.linear.x));
            break;
        }
        case derived_object_msgs::Object::CLASSIFICATION_CAR:
        {
            // return true;
            std::cout << wp << "; " << dist_of_wp_obj << " " << dist_of_wp_ego << " " << dist_of_wp_obj / sqrt(pow(in_obj.object.twist.linear.x, 2) + pow(in_obj.object.twist.linear.y, 2)) << " " << dist_of_wp_ego / m_ego_twist.linear.x << std::endl;
            return (dist_of_wp_obj / sqrt(pow(in_obj.object.twist.linear.x, 2) + pow(in_obj.object.twist.linear.y, 2))) < (dist_of_wp_ego / m_ego_twist.linear.x);
            break;
        }
        case derived_object_msgs::Object::CLASSIFICATION_UNKNOWN:
        {
            // return true;
            return (dist_of_wp_obj < 5.0);
            break;
        }
    }
}


void RasCore::addObjIdToOccupancyWp(const std::vector<int> &in_waypoint, const ras::RasObject &in_obj)
{
    for (const auto &e : in_waypoint)
    {
        m_wp_obj_map[e].emplace_back(in_obj.object.id);
    }
}


/*
search wall waypoint from start to goal
skip passed wapoint
check the collision with object related to the waypoint
then return wall waypoint and object list on the waypoint
*/
int RasCore::findWallWp(std::vector<int> &critical_obj_id_vec)
{
    for (const auto &e : m_wp_obj_map)
    {
        if (e.first < m_brakable_wp) continue;
        // std::cout << e.first << std::endl;
        critical_obj_id_vec.clear();

        for (const auto &obj_id : e.second)
        {
            if (isCollideObstacle(m_obj_map[obj_id], e.first))
            {
                critical_obj_id_vec.emplace_back(obj_id);
                // wall_wp = e.first;
                std::cout << "wall wp is :" << e.first << std::endl;
                return(e.first);
            }
            std::cout << "rejected wall wp is :" << e.first << std::endl;

        }
    }
    return 0;
}


/*
publish object and wall
*/
void RasCore::manageMarkers()
{
    // ROS_INFO("tekaAttendance");
    std::vector<int> erase_key_vec, critical_obj_id_vec;
    ras::RasObjectArray obj_array;
    ras::RasObject wall;
    int wall_wp;

    wall_wp = findWallWp(critical_obj_id_vec);
    // std::cout << "critical object : " << critical_obj_id_vec.size() << std::endl;

    for (auto &e : m_obj_map)
    {
        // erace old object
        if ((ros::Time::now() - e.second.object.header.stamp) > ros::Duration(m_keep_time))
        {
            erase_key_vec.emplace_back(e.first);
            continue;
        }

        if (std::find(critical_obj_id_vec.begin(), critical_obj_id_vec.end(), e.first) != critical_obj_id_vec.end())
        {
            e.second.is_important = true;
            e.second.is_interaction = true;
            std::cout << "critical is :" << e.second.object.id << std::endl;
        }

        else if (e.second.is_touched)
        {
            e.second.is_important = false;
            e.second.is_interaction = true;
        }

        obj_array.objects.emplace_back(e.second);
    }

    // reset is_touched flag if the object appears again from behind
    for (const auto &e : m_wp_obj_map[m_ego_wp])
    {
        m_obj_map[e].is_touched = false;
    }

    // publish
    obj_array.header.stamp = ros::Time::now();
    obj_array.header.frame_id = "map";
    pub_obj.publish(obj_array);

    // std::cout << wall_wp << std::endl;
    if (wall_wp != 0)
    {
        // finally add wall
        wall.object.header.stamp = obj_array.header.stamp;
        wall.object.header.frame_id = obj_array.header.frame_id;
        wall.object.id = obj_array.objects.back().object.id + 1;
        wall.object.pose = m_waypoints[wall_wp];
        wall.object.shape.type = shape_msgs::SolidPrimitive::BOX;
        wall.object.shape.dimensions.emplace_back(0.1);
        wall.object.shape.dimensions.emplace_back(5.0);
        wall.object.shape.dimensions.emplace_back(4.0);
        wall.object.classification = derived_object_msgs::Object::CLASSIFICATION_BARRIER;
        wall.is_interaction = false;
        wall.is_important = true;
        wall.distance = abs(m_ego_wp - wall_wp) * m_wp_interval;
        pub_wall.publish(wall);
    }

    // erace old object
    for (const auto &e : erase_key_vec)
    {
        m_obj_map.erase(e);
    }
}


void RasCore::pubOccupancyWp(const geometry_msgs::Point &in_pose, const int &type)
{
    geometry_msgs::PointStamped point;
    point.point = in_pose;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";

    switch (type)
    {
        case 0:
            pub_wp_obj.publish(point);
            break;
        case 1:
            pub_wp_cross_twist.publish(point);
            break;
        case 2:
            pub_wp_cross_pose.publish(point);
    }
}


/*
the intaractive marker is created for is_interaction=true object
the feedback is also from is_interaction=true object, not from unrelated objects
*/
void RasCore::subShiftCallback(const ras::RasObject &in_msg)
{
    int id = in_msg.object.id;
    if (m_obj_map.find(id) != m_obj_map.end())
    {
        ras::RasObject &obj = m_obj_map[id];
        std::cout << "touched" << std::endl;
        obj.is_touched = !obj.is_touched;
        // manageMarkers();
    }
}
