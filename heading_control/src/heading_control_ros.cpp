#include "heading_control/heading_control_ros.h"

HeadingControlROS::HeadingControlROS(ros::NodeHandle& nh): nh_(nh), monitored_heading_(0), desired_heading_(0), is_enabled_(true)
{
    loadParameters();
    desired_heading_subscriber_ = nh_.subscribe(desired_heading_topic_, 1, &HeadingControlROS::desiredHeadingCallback, this);
    monitored_heading_subscriber_ = nh_.subscribe(monitored_heading_topic_, 1, &HeadingControlROS::monitoredHeadingCallback, this);
    heading_control_switch_service_ = nh.advertiseService("/heading_control_switch", &HeadingControlROS::headingControlSwitch, this);
    operator_cmd_publisher_ = nh_.advertise<nav2d_operator::cmd>("/cmd", 1);
}

HeadingControlROS::~HeadingControlROS()
{
}

void HeadingControlROS::desiredHeadingCallback(const std_msgs::Float32ConstPtr& desired_heading_msg)
{
    desired_heading_ = desired_heading_msg->data;
}

void HeadingControlROS::monitoredHeadingCallback(const std_msgs::Float32ConstPtr& monitored_heading_msg)
{
    monitored_heading_ = monitored_heading_msg->data;
}

void HeadingControlROS::run()
{
    double turn = 0;
    if(is_enabled_)
    {
        turn = heading_control_.computeDirection(monitored_heading_, desired_heading_);
    }

    nav2d_operator::cmd cmd_msg;
    cmd_msg.Turn = turn;
    cmd_msg.Velocity = 0.05; //hardcoded
    cmd_msg.Mode = 0;
    
    operator_cmd_publisher_.publish(cmd_msg);
}

void HeadingControlROS::loadParameters()
{
    std::string desired_heading_topic, monitored_heading_topic;

    nh_.param<std::string>("desired_heading_topic", desired_heading_topic, "/desired_heading");
    nh_.param<std::string>("monitored_headin_topic", monitored_heading_topic, "/monitored_heading");
    desired_heading_topic_ = desired_heading_topic;
    monitored_heading_topic_ = monitored_heading_topic;
    ROS_DEBUG("desired_heading_topic: %s", desired_heading_topic_.c_str());
    ROS_DEBUG("monitored_heading_topic: %s", monitored_heading_topic_.c_str());
}

bool HeadingControlROS::headingControlSwitch(heading_control::Switch::Request  &req, heading_control::Switch::Response &res)
{
    
    res.status = true;
    return true;
}