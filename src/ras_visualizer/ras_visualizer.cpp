#include <ros/ros.h>
#include "ras_visualizer.h"

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

RasVisualizer::RasVisualizer():
    marker_scale(2.0),
    m_vehicle_deceleration(1.96),
    m_beep_timing(6.0),
    m_stop_distance_to_object(9.0),
    m_deceleration_start_distance(50.0)
{
	ros::NodeHandle n;
    server.reset(new interactive_markers::InteractiveMarkerServer("ras_visualizer_node"));

    sub_obj = n.subscribe("managed_objects", 5, &RasVisualizer::subObjCallback, this);
    sub_wall = n.subscribe("wall_object", 5, &RasVisualizer::subWallCallback, this);
    // sub_intervene_type = n.subscribe("intervene_type", 1, &RasVisualizer::subInterveneTypeCallback, this);
    sub_key_input = n.subscribe("rviz_keyboard_input", 1, &RasVisualizer::subButtonInputCallback, this);
    sub_joy_input = n.subscribe("joy", 1, &RasVisualizer::subJoyInputCallback, this);
    sub_odom = n.subscribe("/carla/ego_vehicle/odometry", 5, &RasVisualizer::subOdomCallback, this);
    pub_fb_obj = n.advertise<ras::RasObject>("feedback_object", 5);
    pub_box = n.advertise<jsk_recognition_msgs::BoundingBoxArray>("object_box", 5);
	pub_wall = n.advertise<visualization_msgs::Marker>("wall_marker", 1);
    pub_pictgram = n.advertise<jsk_rviz_plugins::PictogramArray>("pictogram", 5);
    pub_camera_angle = n.advertise<std_msgs::Float32>("carla_camera_angle", 1);
    m_last_touch_time = ros::Time::now();
    // last_wall_time = ros::Time(0);
}


RasVisualizer::~RasVisualizer()
{
    server.reset();
}


void RasVisualizer::subObjCallback(const ras::RasObjectArray &in_obj_array)
{
    server->clear();
    m_important_objects.clear();
    // std::vector<int> new_wall_obj_list;
    // wall_obj_list.clear();

    jsk_recognition_msgs::BoundingBoxArray box_array;
    // jsk_rviz_plugins::PictogramArray pictogram_array;
    // std_msgs::Float32 critical_obj_direction_from_ego;
    // geometry_msgs::Pose critical_obj_pose;
    bool intervene_beep = false;

    for (auto e : in_obj_array.objects)
    {
        if (e.is_interaction)
        {
            createInteractiveMarker(e);
            if (e.is_important)
            {
                // show pictgram for target object
                m_pictogram_list.pictograms.emplace_back(createPictogram(e, 0));
                m_pictogram_list.pictograms.emplace_back(createPictogram(e, 1));
                m_pictogram_list.pictograms.emplace_back(createPictogram(e, 2));

                // for camera rotation
                // critical_obj_pose = Ras::tfTransformer(e.object.pose, e.object.header.frame_id, m_ego_name);
                // critical_obj_direction_from_ego.data = std::atan(critical_obj_pose.position.x /critical_obj_pose.position.y);

                // avoid beep multiple times (each id should appar only one time in one drivng)
                // if (!std::count(wall_obj_list.begin(), wall_obj_list.end(), e.object.id))
                // {
                //     intervene_beep = true;
                // }

                // pub_camera_angle.publish(critical_obj_direction_from_ego);
                m_important_objects.emplace_back(e.object.id);
                // new_wall_obj_list.emplace_back(e.object.id);
            }
        }
        box_array.boxes.emplace_back(createBox(e));
    }

    // if (intervene_beep)
    // {
    //     // system("/home/kuriatsu/Source/catkin_ws/src/ras/src/ras_visualizer/request_intervention &");
    //     sound_client.playWave("/usr/share/sounds/ros_sounds/taionkei.wav");
    // }

    // if (!wall_pictogram.empty() && !wall_marker.empty())
    // {
    //     pictogram_array.pictograms.emplace_back(wall_pictogram[0]);
    //     pub_wall.publish(wall_marker[0]);
    //     wall_pictogram.pop_back();
    //     wall_marker.pop_back();
    // }

    // else
    // {
    //     critical_obj_direction_from_ego.data = 0.0;
    //     pub_camera_angle.publish(critical_obj_direction_from_ego);
    // }

    box_array.header = in_obj_array.header;
    m_pictogram_list.header = in_obj_array.header;

    server->applyChanges();
    pub_box.publish(box_array);
    pub_pictgram.publish(m_pictogram_list);
    m_pictogram_list.pictograms.clear();
}


void RasVisualizer::subWallCallback(const ras::RasObject &in_obj)
{
    bool is_beep = false;
    m_pictogram_list.pictograms.emplace_back(createPictogram(in_obj, 3));
    pub_wall.publish(createMarker(in_obj));
    // wall_pictogram.emplace_back(createPictogram(in_obj, 3));
    // wall_marker.emplace_back(createMarker(in_obj));
    for (auto &important_object_id : m_important_objects)
    {
        if (!std::count(m_beeped_object_history.begin(), m_beeped_object_history.end(), important_object_id))
        {
            is_beep = true;
        }
    }

    if (!is_beep) return;
    float beep_distance = m_deceleration_start_distance + m_beep_timing * m_ego_twist.linear.x;
    // float beep_distance = m_stop_distance_to_object + pow(m_ego_twist.linear.x, 2) / (2 * m_vehicle_deceleration) + m_beep_timing * m_ego_twist.linear.x;
    // float beep_distance = m_stop_distance_to_object + m_deceleration_start_distance + m_beep_timing * m_ego_twist.linear.x;
    ROS_INFO("visualizer: beep distance %f, distance %f", beep_distance, in_obj.distance);
    if ( in_obj.distance < beep_distance)
    // if ( in_obj.distance < beep_distance || in_obj.distance < m_deceleration_start_distance)
    // if ( in_obj.distance < beep_distance || in_obj.distance < m_deceleration_start_distance)
    {
        m_beeped_object_history.insert(m_beeped_object_history.end(), m_important_objects.begin(), m_important_objects.end());
        sound_client.playWave("/usr/share/sounds/ros_sounds/taionkei.wav");
    }
}


jsk_recognition_msgs::BoundingBox RasVisualizer::createBox(const ras::RasObject &in_obj)
{
    jsk_recognition_msgs::BoundingBox marker;

    marker.header = in_obj.object.header;
    marker.label = in_obj.object.id;

    marker.pose = in_obj.object.pose;
    marker.dimensions.x = in_obj.object.shape.dimensions[0];
    marker.dimensions.y = in_obj.object.shape.dimensions[1];
    marker.dimensions.z = in_obj.object.shape.dimensions[2];

    if (in_obj.object.classification == derived_object_msgs::Object::CLASSIFICATION_CAR || in_obj.object.classification == derived_object_msgs::Object::CLASSIFICATION_BARRIER)
    {
        marker.pose.position.z += in_obj.object.shape.dimensions[2] / 2;
    }

    if (in_obj.is_important)
    {
        marker.value = 100.0;
    }
    else
    {
        marker.value = 50.0;
    }
    return marker;
}


visualization_msgs::Marker RasVisualizer::createMarker(const ras::RasObject &in_obj)
{
    visualization_msgs::Marker marker;

    marker.header = in_obj.object.header;
    marker.ns = "ras";
    marker.id = in_obj.object.id;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = in_obj.object.pose;
    marker.scale.x = in_obj.object.shape.dimensions[0];
    marker.scale.y = in_obj.object.shape.dimensions[1];
    marker.scale.z = in_obj.object.shape.dimensions[2];

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.3;

    marker.lifetime = ros::Duration(0.1);
    return marker;
}


void RasVisualizer::createInteractiveMarker(ras::RasObject &in_obj)
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
    int_marker.pose.position.x = in_obj.object.pose.position.x;
    int_marker.pose.position.y = in_obj.object.pose.position.y;

    setMarkerControl(int_marker, in_obj);

	server->insert(int_marker);
	server->setCallback(int_marker.name, boost::bind(&RasVisualizer::intMarkerCallback, this, _1));
}


void RasVisualizer::setMarkerControl(visualization_msgs::InteractiveMarker &int_marker, const ras::RasObject &in_obj)
{
	visualization_msgs::InteractiveMarkerControl control;

	control.always_visible  = true;
	control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

    setMarkerToMarkerControl(control, in_obj);
    int_marker.controls.push_back(control);
}


void RasVisualizer::setMarkerToMarkerControl(visualization_msgs::InteractiveMarkerControl &control, const ras::RasObject &in_obj)
{
    visualization_msgs::Marker marker;

    marker.ns = "ras";
    marker.id = in_obj.object.id;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = marker_scale*in_obj.object.shape.dimensions[0];
    marker.scale.y = marker_scale*in_obj.object.shape.dimensions[1];
    marker.scale.z = marker_scale*in_obj.object.shape.dimensions[2];

	if (in_obj.is_important)
	{
		marker.color.r = 1;
		marker.color.g = 0;
	}
	else
	{
		marker.color.r = 0;
		marker.color.g = 1;
	}
    marker.color.b = 0;
    marker.color.a = 0.0;

    // marker.lifetime = ros::Duration(2.0);
    control.markers.push_back(marker);
}


jsk_rviz_plugins::Pictogram RasVisualizer::createPictogram(const ras::RasObject &in_obj, const int &type)
{
    jsk_rviz_plugins::Pictogram pictogram;
    geometry_msgs::Pose arrow_pose;
    float arrow_len;
    std::map <int, std::string> message
    {
        {0,"fa-question"},
        {4,"fa-user"},
        {6, "fa-car"},
        {10, "block"}
        // {10, "traffic-cone"}
    };

    pictogram.header.stamp = ros::Time::now();
    pictogram.color.a = 1.0;
    pictogram.ttl = 0.1;

    switch (type)
    {
        case 0:
        {

            pictogram.header.frame_id = "base_link";
            pictogram.pose = Ras::tfTransformer(in_obj.object.pose, in_obj.object.header.frame_id, "base_link");
            pictogram.pose.position.z += 2.0;
            pictogram.pose.orientation.x = 0.0;
            pictogram.pose.orientation.y = 1.0;
            pictogram.pose.orientation.z = 0.0;
            pictogram.pose.orientation.w = -1.0;
            pictogram.color.r = 1.0;
            pictogram.color.g = 0.0;
            pictogram.color.b = 0.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::JUMP;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = "fa-angle-double-down";
            pictogram.speed = 5;
            pictogram.size = 3;
            break;
        }

        case 1:
        {

            pictogram.header.frame_id = "base_link";
            pictogram.pose = Ras::tfTransformer(in_obj.object.pose, in_obj.object.header.frame_id, "base_link");
            pictogram.pose.position.z += 5.0;
            pictogram.pose.orientation.x = 0.0;
            pictogram.pose.orientation.y = 1.0;
            pictogram.pose.orientation.z = 0.0;
            pictogram.pose.orientation.w = -1.0;
            pictogram.color.r = 1.0;
            pictogram.color.g = 1.0;
            pictogram.color.b = 1.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::ADD;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = message[in_obj.object.classification];
            pictogram.size = 6;
            // pictogram.character = message[in_obj.object.classification];
            break;
        }

        case 2:
        {

            pictogram.header.frame_id = "base_link";
            geometry_msgs::Pose arrow_pose = in_obj.object.pose;
            arrow_pose = Ras::tfTransformer(arrow_pose, in_obj.object.header.frame_id, "base_link");
            arrow_len = sqrt(pow(arrow_pose.position.x, 2) + pow(arrow_pose.position.y, 2));
            arrow_pose.position.x -= 5.0;
            pictogram.pose.orientation.x = 0.0;
            pictogram.pose.orientation.y = 0.0;
            pictogram.pose.orientation.z = -arrow_pose.position.y / (arrow_len * 2);
            pictogram.pose.orientation.w = - arrow_pose.position.x / (arrow_len);
            pictogram.pose.position.x = 5.0;
            pictogram.pose.position.y = 0.0;
            pictogram.pose.position.z = 0.0;
            pictogram.color.r = 1.0;
            pictogram.color.g = 0.0;
            pictogram.color.b = 0.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::ADD;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = "fa-long-arrow-alt-up";
            pictogram.size = 3;
            break;
        }

        case 3:
        {
            geometry_msgs::Quaternion quat = in_obj.object.pose.orientation;
            pictogram.header.frame_id = "map";
            pictogram.pose.position = in_obj.object.pose.position;
            pictogram.pose.position.z = 0.5;
            pictogram.pose.orientation.x = quat.z * 0.7 + quat.x * 0.7;
            pictogram.pose.orientation.y = -quat.w * 0.7 + quat.y * 0.7;
            pictogram.pose.orientation.z = - quat.x * 0.7 + quat.z * 0.7;
            pictogram.pose.orientation.w = quat.y * 0.7 + quat.w * 0.7;
            pictogram.color.r = 1.0;
            pictogram.color.g = 0.0;
            pictogram.color.b = 0.0;
            pictogram.action = jsk_rviz_plugins::Pictogram::ADD;
            pictogram.mode = jsk_rviz_plugins::Pictogram::PICTOGRAM_MODE;
            pictogram.character = message[in_obj.object.classification];
            pictogram.size = 5;
            break;
        }
    }

    return pictogram;
}

void RasVisualizer::intMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    ras::RasObject feedback_obj;
    std::istringstream sis;
    // remove bounsing when touched
    if (ros::Time::now() - m_last_touch_time < ros::Duration(0.5))
    {
        return;
    }
    std::istringstream(feedback->marker_name) >> feedback_obj.object.id;
    pub_fb_obj.publish(feedback_obj);
    m_last_touch_time = ros::Time::now();
}

void RasVisualizer::subButtonInputCallback(const std_msgs::String &in_key)
{
    ras::RasObject feedback_obj;
    // if (intervene_type != 1) return;
    std::cout << in_key.data << (in_key.data != "Return") << std::endl;
    if (in_key.data != "Return") return;
    if (!m_important_objects.empty())
    {
        for (const auto &e: m_important_objects)
        {
            feedback_obj.object.id = e;
            pub_fb_obj.publish(feedback_obj);
        }
        m_last_intervened_objects = m_important_objects;
    }
    else
    {
        for (const auto &e: m_last_intervened_objects)
        {
            feedback_obj.object.id = e;
            pub_fb_obj.publish(feedback_obj);
        }
    }
}


void RasVisualizer::subJoyInputCallback(const sensor_msgs::Joy &in_joy)
{
    static int last_joy_input = 0; // latch input
    ras::RasObject feedback_obj;

    if (in_joy.buttons[23] == 1 && last_joy_input == 0)
    {
        if (!m_important_objects.empty())
        {
            for (const auto &e: m_important_objects)
            {
                feedback_obj.object.id = e;
                pub_fb_obj.publish(feedback_obj);
            }
            m_last_intervened_objects = m_important_objects;
        }
        else
        {
            for (const auto &e: m_last_intervened_objects)
            {
                feedback_obj.object.id = e;
                pub_fb_obj.publish(feedback_obj);
            }
        }
    }
    last_joy_input = in_joy.buttons[23];
}

// void RasVisualizer::subInterveneTypeCallback(const std_msgs::Int32 &in_intervene_type)
// {
//     intervene_type = in_intervene_type.data;
// }

void RasVisualizer::subOdomCallback(const nav_msgs::Odometry &in_odom)
{
    m_ego_twist = in_odom.twist.twist;
}
