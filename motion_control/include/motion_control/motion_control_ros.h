#ifndef MOTION_CONTROL_ROS_H
#define MOTION_CONTROL_ROS_H

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <motion_control/command.h>
#include <motion_control/Switch.h>

#include <motion_control/motion_control.h>


class MotionControlROS
{

public:
    // Constructor / destructor
    MotionControlROS(ros::NodeHandle&);
    ~MotionControlROS();


private:
    // ROS sub/pub
    ros::NodeHandle nh_;
    ros::Subscriber motion_command_subscriber_;
    ros::ServiceServer motion_control_switch_service_;
    ros::Publisher velocity_command_publisher_;
    ros::Publisher trajectory_publisher_;
    ros::Publisher plan_publisher_;
    ros::Publisher cost_publisher_;

    // ROS tf
    tf::TransformListener tf_listener;
    tf2_ros::Buffer tf2_buffer;
    tf2_ros::TransformListener tf2_listener;

    // costmap params
    costmap_2d::Costmap2DROS* local_map;
    costmap_2d::Costmap2D* costmap;
    double raster_size;

    // Parameters
    std::string motion_command_topic_, velocity_command_topic_, robot_frame_, odometry_frame_;;

    // robot motion monitoring
    MotionControl motion_control_;
    bool is_enabled_;

    // desired params
    double desired_velocity_;
    double desired_direction_;
    int drive_mode_;

    // current robot state
    double current_velocity_;
    double current_direction_;

    // Members
    void loadParameters();
    void motionCommandCallback(const motion_control::CommandConstPtr& motion_cmd_msg);
    bool motionControlSwitch(motion_control::Switch::Request  &req, motion_control::Switch::Response &res);

};

#endif
