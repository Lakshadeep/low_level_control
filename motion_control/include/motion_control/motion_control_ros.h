#ifndef MOTION_CONTROL_ROS_H
#define MOTION_CONTROL_ROS_H

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "motion_control/motion_control.h"
#include "motion_control/Switch.h"
#include "motion_control/Command.h"


class MotionControlROS
{

public:
    // Constructor / destructor
    MotionControlROS(ros::NodeHandle&);
    ~MotionControlROS();


private:
    // ROS
    ros::NodeHandle nh_;
    ros::Subscriber motion_command_subscriber_;
    ros::ServiceServer motion_control_switch_service_;
    ros::Publisher velocity_command_publisher_;

    // Parameters
    std::string motion_command_topic_, velocity_command_topic_;

    // robot motion monitoring
    MotionControl motion_control_;
    bool is_enabled_;
    
    // Members
    void loadParameters();
    void motionCommandCallback(const motion_control::CommandConstPtr& motion_cmd_msg);
    bool motionControlSwitch(motion_control::Switch::Request  &req, motion_control::Switch::Response &res);

};

#endif
