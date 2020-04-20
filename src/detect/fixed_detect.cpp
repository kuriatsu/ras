#include <ros/ros.h>

// ros msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "swipe_obstacles/detected_obstacle.h"
#include "swipe_obstacles/detected_obstacle_array.h"
#include "swipe_obstacles/closest_obstacle.h"
#include <tf/transform_listener.h>
#include "std_msgs/Int32.h"

// file read
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

struct read_obstacle
{
    swipe_obstacles::detected_obstacle detected_obstacle;
    // swipe_obstacles::closest_obstacle closest_obstacle;
    bool auto_move;
    float stop_time;
    float move_dist;
};

class SwipeDetectorFixed
{
private:
    // pub sub
    ros::Publisher pub_obstacle_pose;
    // ros::Publisher pub_closest_obstacle;
    ros::Publisher pub_erase_signal;

    ros::Subscriber sub_twist;

    // tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // ros::Publisher test_obstacle_pose;
    // ros::Publisher test_transform_origin;
    // tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    ros::Subscriber sub_vehicle_pose;
    ros::Subscriber sub_waypoint_callback;

    // obstacle container setting
    const static int vector_size = 20;
    std::vector<read_obstacle> read_obstacle_vec;

    // management information
    ros::Timer pub_timer;
    float pub_rate;
    int keep_time;
    ros::Time last_pub_time;
    tf::TransformListener tf_listener;

    int round;
    geometry_msgs::Pose vehicle_pose;
    uint32_t next_waypoint_flag;

    ros::Time stoped_time;
    int closest_obj_id;
    bool closest_obj_is_new = false;
    bool auto_move;

public:
    SwipeDetectorFixed();

private:
    void readFile(const std::string &file_name);
    void pubTimerCallback(const ros::TimerEvent&);
    void waypointCallback(const std_msgs::Int32 &in_msg);
    void twistCallback(const geometry_msgs::TwistStamped &in_msg);

    geometry_msgs::Pose tfTransformer(const geometry_msgs::Pose &in_pose, const std::string &current_frame_id, const std::string &target_frame_id);
    double quatToRpy(const geometry_msgs::Quaternion &quat);

};


SwipeDetectorFixed::SwipeDetectorFixed()
{
    std::string file_name;

    ros::NodeHandle n;
    pub_obstacle_pose = n.advertise<swipe_obstacles::detected_obstacle_array>("/detected_obstacles", 5);
    // pub_closest_obstacle = n.advertise<swipe_obstacles::closest_obstacle>("/closest_obstacle", 5);
    pub_erase_signal = n.advertise<std_msgs::Int32>("/swipe_erase_signal", 5);
    sub_twist = n.subscribe("/twist_cmd", 1, &SwipeDetectorFixed::twistCallback, this);
    sub_waypoint_callback = n.subscribe("/closest_waypoint", 5, &SwipeDetectorFixed::waypointCallback, this);
    // tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    // test_transform_origin = n.advertise<geometry_msgs::PoseStamped>("/test_transform_origin", 5);
    // test_obstacle_pose = n.advertise<geometry_msgs::PoseStamped>("/test_obstacle_pose", 5);
    // tf_check%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    n.getParam("/detector_fixed_node/file_name", file_name);
    n.getParam("/detector_fixed_node/start_round", round);
    n.getParam("/detector_fixed_node/keep_time", keep_time);
    n.getParam("/detector_fixed_node/pub_rate", pub_rate);

    // std::cout << file_name << std::endl;
    read_obstacle_vec.reserve(vector_size);
    readFile(file_name);
    ros::Duration(1).sleep();
    pub_timer = n.createTimer(ros::Duration(pub_rate), &SwipeDetectorFixed::pubTimerCallback, this);
}


void SwipeDetectorFixed::waypointCallback(const std_msgs::Int32 &in_msg)
{
    if (in_msg.data == 0)
    {
        next_waypoint_flag = 1;
    }
    else if(in_msg.data == 1)
    {
        next_waypoint_flag = 2;
    }
    else if(in_msg.data == next_waypoint_flag)
    {
        next_waypoint_flag = 0;
        round++;
        std::cout << "round : " << round << std::endl;
    }
}


void SwipeDetectorFixed::readFile(const std::string &file_name)
{
    read_obstacle read_obstacle;
    std::string line;
    std::ifstream ifs(file_name);

    if (!ifs == 2)
    {
        ROS_ERROR("Cannot Open File !");
        return;
    }

    // skip first line
    std::getline(ifs, line);
    //read csv file
    while(std::getline(ifs, line))
    {
        std::istringstream stream(line);
        std::string value;
        std::vector<std::string> result;

        while(std::getline(stream, value, ','))
        {
            result.push_back(value);
        }

        read_obstacle.detected_obstacle.id = std::stoi(result.at(0));
        read_obstacle.detected_obstacle.round = std::stoi(result.at(1));
        read_obstacle.detected_obstacle.pose.position.x = std::stof(result.at(2));
        read_obstacle.detected_obstacle.pose.position.y = std::stof(result.at(3));
        read_obstacle.detected_obstacle.pose.position.z = std::stof(result.at(4));
        read_obstacle.detected_obstacle.pose.orientation.x = std::stof(result.at(5));
        read_obstacle.detected_obstacle.pose.orientation.y = std::stof(result.at(6));
        read_obstacle.detected_obstacle.pose.orientation.z = std::stof(result.at(7));
        read_obstacle.detected_obstacle.pose.orientation.w = std::stof(result.at(8));
        read_obstacle.detected_obstacle.shift_x = std::stof(result.at(9));
        read_obstacle.detected_obstacle.shift_y = std::stof(result.at(10));
        read_obstacle.detected_obstacle.label = result.at(11);
        read_obstacle.detected_obstacle.score = std::stof(result.at(12));
        read_obstacle.detected_obstacle.header.frame_id = result.at(13);

        // read_obstacle.closest_obstacle.id = std::stoi(result.at(0));
        // read_obstacle.closest_obstacle.brief_stop = std::stoi(result.at(14));
        read_obstacle.auto_move = std::stoi(result.at(14));
        read_obstacle.stop_time = std::stof(result.at(15));
        read_obstacle.move_dist = std::stof(result.at(16));

        read_obstacle_vec.push_back(read_obstacle);
        // ROS_INFO_STREAM(read_obstacle);
    }
}


void SwipeDetectorFixed::pubTimerCallback(const ros::TimerEvent&)
{
    swipe_obstacles::detected_obstacle_array out_array;
    geometry_msgs::Pose pose_from_velodyne;

    int search_closest_obj_array_idx, search_closest_obj_vec_idx;
    float search_closest_obj_dist = 100.0;
    ros::Duration stop_time, stopping_time;

    int pub_flag=0;
    std_msgs::Int32 erase_signal;

    double yaw;
    swipe_obstacles::detected_obstacle move_obj;

    for(auto i=read_obstacle_vec.begin(); i!=read_obstacle_vec.end(); i++)
    {
        if(i->detected_obstacle.round == round)
        {
            pose_from_velodyne = tfTransformer(i->detected_obstacle.pose, i->detected_obstacle.header.frame_id, "/velodyne");
            if(0.0 < pose_from_velodyne.position.x && pose_from_velodyne.position.x < 10.0)
            {
                // ROS_INFO_STREAM(pose_from_velodyne.position);
                if(-5.0 < pose_from_velodyne.position.y && pose_from_velodyne.position.y < 5.0)
                {
                    // add obstacle to array
                    i->detected_obstacle.detected_time = ros::Time::now();
                    out_array.obstacles.push_back(i->detected_obstacle);
                    // find closest obstacle
                    if(pose_from_velodyne.position.x < search_closest_obj_dist)
                    {
                        search_closest_obj_array_idx = out_array.obstacles.size() - 1;
                        search_closest_obj_vec_idx = std::distance(read_obstacle_vec.begin(), i);
                        search_closest_obj_dist = pose_from_velodyne.position.x;
                    }
                    pub_flag = 1;
                }
            }
        }
    }

    if(pub_flag)
    {
        // check new closest obj info
        if (closest_obj_id != read_obstacle_vec.at(search_closest_obj_vec_idx).detected_obstacle.id)
        {
            closest_obj_is_new = true;
            closest_obj_id = read_obstacle_vec.at(search_closest_obj_vec_idx).detected_obstacle.id;
            auto_move = read_obstacle_vec.at(search_closest_obj_vec_idx).auto_move;
        }

        // auto shifting
        stop_time = ros::Duration(read_obstacle_vec.at(search_closest_obj_vec_idx).stop_time);
        stopping_time = ros::Time::now() - stoped_time;
        std::cout << "stopping_time : " << stopping_time << std::endl;

        // if(stop_time + ros::Duration(5.0) <= stopping_time && stopping_time <= stop_time + ros::Duration(8.0)) // for test
        if(stop_time <= stopping_time && stopping_time <= stop_time + ros::Duration(3.0))
        {
            yaw = quatToRpy(read_obstacle_vec.at(search_closest_obj_vec_idx).detected_obstacle.pose.orientation);
            // out_array.obstacles.at(search_closest_obj_array_idx).pose.position.x += ((stopping_time - stop_time).toSec() * 0.5 * read_obstacle_vec.at(search_closest_obj_vec_idx).move_dist) * sin(2*M_PI+yaw);
            // out_array.obstacles.at(search_closest_obj_array_idx).pose.position.y += ((stopping_time - stop_time).toSec() * 0.5 * read_obstacle_vec.at(search_closest_obj_vec_idx).move_dist) * cos(2*M_PI+yaw);
            read_obstacle_vec.at(search_closest_obj_vec_idx).detected_obstacle.pose.position.x += (read_obstacle_vec.at(search_closest_obj_vec_idx).move_dist / 3.0) * sin(M_PI-yaw) * pub_rate;
            read_obstacle_vec.at(search_closest_obj_vec_idx).detected_obstacle.pose.position.y += (read_obstacle_vec.at(search_closest_obj_vec_idx).move_dist / 3.0) * cos(M_PI-yaw) * pub_rate;
            // ROS_INFO_STREAM(read_obstacle_vec.at(search_closest_obj_vec_idx).detected_obstacle.pose.position);

        }

        // publish array
        out_array.header.frame_id = "map";
        out_array.header.stamp = ros::Time::now();
        pub_obstacle_pose.publish(out_array);

        last_pub_time = ros::Time::now();
    }

    if(ros::Time::now() - last_pub_time > ros::Duration(keep_time))
    {
        erase_signal.data = 1;
        pub_erase_signal.publish(erase_signal);
    }
}


double SwipeDetectorFixed::quatToRpy(const geometry_msgs::Quaternion &quat)
{
    tf::Quaternion tf_quat;
    double roll, pitch, yaw;
    quaternionMsgToTF(quat, tf_quat);
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}


geometry_msgs::Pose SwipeDetectorFixed::tfTransformer(const geometry_msgs::Pose &current_pose, const std::string &current_frame_id, const std::string &target_frame_id)
{
    tf::Pose current_tf;
    tf::StampedTransform transform;
    tf::Pose transformed_tf;
    geometry_msgs::Pose transformed_pose;
    geometry_msgs::PoseStamped transform_origin;

    try{
        tf_listener.waitForTransform(current_frame_id, target_frame_id,  ros::Time(0), ros::Duration(1.0));
        // current_frame_id　から　target_frame_id　への座標変換
        tf_listener.lookupTransform(target_frame_id, current_frame_id, ros::Time(0), transform);

    }catch (tf::TransformException &ex)  {
        ROS_ERROR("%s", ex.what());
        //ros::Duration(1.0).sleep();
    }

    tf::poseMsgToTF(current_pose, current_tf);
    transformed_tf = transform * current_tf;
    tf::poseTFToMsg(transformed_tf, transformed_pose);

    return transformed_pose;
}


void SwipeDetectorFixed::twistCallback(const geometry_msgs::TwistStamped &in_msg)
{
    // if(closest_obj_is_new) // for test
    if(auto_move && closest_obj_is_new && in_msg.twist.linear.x == 0.0)
    {
        stoped_time = ros::Time::now();
        // std::cout << "timer start: " << stoped_time << std::endl;
        closest_obj_is_new = false;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "swipe_detector_fixed1_node");

    ROS_INFO("Initializing detector...");
    // ros::Duration(0.1).sleep();
    SwipeDetectorFixed swipe_detector_fixed;

    ROS_INFO("detector ready...");

    ros::spin();
    return 0;
}
