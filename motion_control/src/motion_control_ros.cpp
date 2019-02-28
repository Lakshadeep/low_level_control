#include "motion_control/motion_control_ros.h"

MotionControlROS::MotionControlROS(ros::NodeHandle& nh): nh_(nh), is_enabled_(false)
{
    loadParameters();
    motion_command_subscriber_ = nh_.subscribe(motion_command_topic_, 1, &MotionControlROS::motionCommandCallback, this);
    motion_control_switch_service_ = nh.advertiseService("/motion_control_switch", &MotionControlROS::motionControlSwitch, this);
    velocity_command_publisher_ = nh_.advertise<motion_control::Command>(velocity_command_topic_, 1);
}

MotionControlROS::~MotionControlROS()
{
}

void MotionControlROS::motionCommandCallback(const motion_control::CommandConstPtr& motion_cmd_msg)
{
}

void MotionControlROS::loadParameters()
{
    std::string motion_command_topic, velocity_command_topic;

    nh_.param<std::string>("motion_command_topic", motion_command_topic, "/motion_command");
    nh_.param<std::string>("velocity_command_topic", velocity_command_topic, "/ropod/cmd_vel");
    
    motion_command_topic_ = motion_command_topic;
    velocity_command_topic_ = velocity_command_topic;

    ROS_DEBUG("motion_command_topic: %s", motion_command_topic_.c_str());
    ROS_DEBUG("velocity_command_topic: %s", velocity_command_topic_.c_str());

    // double kp;
    // nh_.param<double>("proportional_gain", kp, 2.0);
    // motion_control_.setProportionalGain(kp);
    // ROS_DEBUG("proportional_gain: %f", kp);
}

bool MotionControlROS::motionControlSwitch(motion_control::Switch::Request  &req, motion_control::Switch::Response &res)
{
    is_enabled_ = req.enable;
    res.status = true;
    return true;
}