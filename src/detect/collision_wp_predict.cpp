#include <ros/ros.h>
#include "collision_wp_predict.h"

CollisionWpPredict::CollisionWpPredict()
{
	ros::NodeHandle n;

	server_callback = boost::bind(&RasCore::callbackDynamicReconfigure, this, _1, _2);
	server.setCallback(server_callback);

	sub_trajectory = n.subscribe("/lane_waypoints_array", 5, &CollisionWpPredict::subTrajectoryCallback, this);
	sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &CollisionWpPredict::subOdomCallback, this);
	sub_obj = n.subscrive("/managed_objects", 5, &CollisionWpPredict::subObjCallback, this);

	pub_wall = n.advertise<ras::RasWall>("/wall", 1);
	pub_wp_obj = n.advertise<geometry_msgs::PointStamped>("/obj_wp", 5);
	pub_wp_cross = n.advertise<geometry_msgs::PointStamped>("/crossed_wp", 5);
}


void RasCore::callbackDynamicReconfigure(ras::rasConfig &config, uint32_t lebel)
{
	m_max_vision = config.max_vision_range;
	m_min_vision = config.min_vision_range;

}

// get trajectory
void CollisionWpPredict::subTrajectoryCallback(const autoware_msgs::LaneArray &in_array)
{
	for (const auto &itr : in_array.lanes[0].waypoints)
	{
		m_wps_vec.emplace_back(itr.pose.pose);
	}
	m_wp_interval = sqrt(pow(m_wps_vec[0].position.x - m_wps_vec[1].position.x, 2) + pow(m_wps_vec[0].position.y - m_wps_vec[1].position.y, 2));
}


// get ego vehicle info(pose, twist, closest_wp) and save
void CollisionWpPredict::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
	float min_dist = 9.0, dist; // use when find closest wp from ego vehicle
    int ego_wp = 0; // closest waypoint from ego_vehicle

	// ego_vehcle info
	m_ego_pose = in_odom.pose.pose;
	m_ego_twist = in_odom.twist.twist;

	// find closest waypoint from m_waypoints
	for (size_t i = 0; i < m_wps_vec.size(); i++)
	{
		dist = Ras::calcDistOfPoints(m_ego_pose.position, m_wps_vec[i].position);
		if (min_dist > dist)
		{
			ego_wp = i;
			min_dist = dist;
		}
	}

	// ego_vehicle way_point
    m_ego_wp = ego_wp;
    m_brakable_wp = ego_wp + (int)((pow(m_ego_twist.linear.x * 3.6, 2) / (254 * 0.7)) / m_wp_interval); // wp to stop with -0.7G
}


void CollisionWpPredict::subObjCallback(const ras::RasObjectArray &in_obj_array)
{
	std::vector<int> critical_obj_id_vec;
	int wall_wp_id;
	m_wp_obj_map.clear();

	// Avoid crash caused by trying to find wp before getting waypoints and ego vehicle info
	if (m_wps_vec.empty() || m_ego_wp == 0)
	{
		ROS_ERROR("waypoint or ego odometry is not subscrived yet");
		return;
	}

	// get waypoint of object and calc waypoint-obstacle map
	for (const auto &e : in_obj_array)
	{
		calcOccupancyWp(findWpOfObj(e), e);
	}

	// get waypoint id to put the wall
	findWallWp(wall_wp_id, critical_obj_id_vec);

	// publish wall
	if (wall_wp != 0)
	{
		RasWall wall;
		wall.header.stamp = in_obj_array.header.stamp;
		wall.header.frame_id  = in_obj_array.header.frame_id;
		wall.pose = m_wps_vec[wall_wp];
		wall.obj_id_vec = critical_obj_id_vec;
		pub_wall.publish(wall);
	}
}


// find cross or perpendicular waypoint of in_obj
std::vector<int> CollisionWpPredict::findWpOfObj(ras::RasObject &in_obj)
{
    std::vector<int> out_wp_vec; // waypoint list of object

    switch(in_obj.object.classification)
    {
        case derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN:
        {
            float min_dist = 100, dist;
            int close_wp, perp_wp; // closest and perpendicular waypoint from pedestrian

	        // find closest waypoint from object
            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            {
                dist = Ras::calcDistOfPoints(itr->position, in_obj.object.pose.position);
                if (dist < min_dist)
                {
                    min_dist = dist;
                    close_wp = std::distance(m_wps_vec.begin(), itr);
                }
            }

			// check wheather the closestwp-pedestrian and pedestrian is same direction
            CalcVector obj_close_wp_vec(in_obj.object.pose.position, m_wps_vec[close_wp].position);
            CalcVector obj_vec(in_obj.object.pose);
            if (isSameDirection(obj_close_wp_vec, obj_vec, 0.7))
            {
             	out_wp_vec.emplace_back(close_wp);
                pubOccupancyWp(m_wps_vec[close_wp].position, 0);
            }

            // find cross waypoint of pedestrian (perpendicular or cross)
            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr++)
            {
                perp_wp = std::distance(m_wps_vec.begin(), itr);

                if(perp_wp == 0) continue;
                if(perp_wp == close_wp) continue;

				// check wether the wp and close_wp is in the same direction or perpendicular
                CalcVector obj_perpwp_vec(in_obj.object.pose.position, itr->position);
                if (isPerpendicular(obj_closewp_vec, obj_perpwp_vec, 0.05) || isSameDirection(obj_closewp_vec, obj_perpwp_vec, 0.8));
                {
					// check wether the path and wp is perpendicular
					CalcVector path_vec((itr-1)->position, itr->position);
                    if (isPerpendicular(path_vec, obj_perpwp_vec, 0.05) && isSameDirection(obj_vec, obj_perpwp_vec, 0.7))
                    {
                     	out_wp_vec.emplace_back(perp_wp);
                        pubOccupancyWp(m_wps_vec[perp_wp].position, 1);
                    }
                }
            }
            break;
        }

        case derived_object_msgs::Object::CLASSIFICATION_CAR:
        {
            float obj_vec_x, obj_vec_y, obj_wp_vec_x, obj_wp_vec_y, inner_prod, dist_of_wp_obj;

			CalcVector obj_vec(in_obj.object.pose);
            for (auto itr = m_wps_vec.begin(); itr != m_wps_vec.end(); itr ++)
            {
				// check wether the wp is in the same direction with object
                CalcVector obj_wp_vec(in_obj.object.pose.position, itr->position);
                if (isSameDirection(obj_vec, obj_wp_vec, 0.999))
                {
                    pubOccupancyWp(m_wps_vec[std::distance(m_wps_vec.begin(), itr)].position, 1);
                 	out_wp_vec.emplace_back(std::distance(m_wps_vec.begin(), itr));
                    // break;
                }
            }
            break;
        }
    }
    return out_wp_vec;
}


// create wp-object map
void CollisionWpPredict::calcOccupancyWp(const std::vector<int> &in_wp_vec, const ras::RasObject &in_obj)
{
    for (const auto &e : in_wp_vec)
    {
        m_wp_obj_map[e].emplace_back(in_obj.object.id);
    }
}


// find waypoint to put wall
void CollisionWpPredict::findWallWp(int &wall_wp_id, std::vector<int> &critical_obj_id_vec)
{
    for (const auto &e : m_wp_obj_map)
    {
        if (e.first < m_brakable_wp) continue; //skip wp closer than stoppable distance

        for (const auto &obj_id : e.second)
        {
            if (isCollideObstacle(m_obj_map[obj_id], e.first))
            {
                critical_obj_id_vec.emplace_back(obj_id);
                wall_wp_id = e.first;
            }
        }
    }
    return;
}


bool CollisionWpPredict::isSameDirection(const CalcVector &vec_1, const CalcVector &vec_2, const float &thres)
{
    // is close wp same direction with the object
    float inner_prod = vec_1.x * vec_2.x + vec_1.y * vec_2.y;
    return (inner_prod > vec_1.len * vec_2.len * thres);
}


bool CollisionWpPredict::isPerpendicular(const CalcVector &vec_1, const CalcVector &vec_2, const float &thres)
{
    float inner_prod = vec_1.x * vec_2.x + vec_1.y * vec_2.y;
    return (fabs(inner_prod) < vec_1.len * vec_2.len * thres);
}


// check wether the critical object will collide with ego_vehicle
bool CollisionWpPredict::isCollideObstacle(const ras::RasObject &in_obj, const int &wp)
{
    float dist_of_wp_ego, dist_of_wp_obj;
    dist_of_wp_ego = (wp - m_ego_wp) * m_wp_interval;
    dist_of_wp_obj = Ras::calcDistOfPoints(in_obj.object.pose.position, m_wps_vec[wp].position);

	// wall waypoint is further than max_vision or object is already touched by human
    if (wp > m_ego_wp + m_max_vision / m_wp_interval || in_obj.is_touched)
        return false;

    switch (in_obj.object.classification)
    {
		// pedestrian is closer to the waypoint than ego_vehcile (distance and time)
        case derived_object_msgs::Object::CLASSIFICATION_PEDESTRIAN:
        {
            return (dist_of_wp_obj < dist_of_wp_ego || dist_of_wp_obj / in_obj.object.twist.linear.x < dist_of_wp_ego / m_ego_twist.linear.x);
            break;
        }
		// car is closer to the waypoint than ego_vehcile (time) , remove behind waypoints
        case derived_object_msgs::Object::CLASSIFICATION_CAR:
        {
            return (dist_of_wp_ego > 0 && dist_of_wp_obj / in_obj.object.twist.linear.x < dist_of_wp_ego / m_ego_twist.linear.x);
            break;
        }
    }
}


// to check that this algorithm can find waypoint properly
void CollisionWpPredict::pubOccupancyWp(const geometry_msgs::Point &in_pose, const int &type)
{
    geometry_msgs::PointStamped point;
    point.point = in_pose;
    point.header.stamp = ros::Time::now();
    point.header.frame_id = "map";

    switch (type)
    {
		// closest waypoint
        case 0:
            pub_wp_obj.publish(point);
            break;
		// cross waypoint
        case 1:
            pub_wp_cross.publish(point);
            break;
    }
}
