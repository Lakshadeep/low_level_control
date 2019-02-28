#include "motion_control/motion_control_ros.h"

MotionControlROS::MotionControlROS(ros::NodeHandle& nh): nh_(nh), is_enabled_(false), tf2_buffer(), tf2_listener(tf2_buffer), 
tf_listener(ros::Duration(0.5))
{
    loadParameters();

    // subscribers, publishers and services
    motion_command_subscriber_ = nh_.subscribe(motion_command_topic_, 1, &MotionControlROS::motionCommandCallback, this);
    motion_control_switch_service_ = nh.advertiseService("/motion_control_switch", &MotionControlROS::motionControlSwitch, this);
    velocity_command_publisher_ = nh_.advertise<motion_control::Command>(velocity_command_topic_, 1);

    // Create the local costmap
    local_map = new costmap_2d::Costmap2DROS(std::string("local_map"), tf_listener);
    raster_size = local_map->getCostmap()->getResolution();

    // Apply tf_prefix to all used frame-id's
    robot_frame_ = tf_listener.resolve(robot_frame_);
    odometry_frame_ = tf_listener.resolve(odometry_frame_);
}

MotionControlROS::~MotionControlROS()
{
}

void MotionControlROS::motionCommandCallback(const motion_control::CommandConstPtr& motion_cmd_msg)
{
    if (motion_cmd_msg->turn < -1 || motion_cmd_msg->turn > 1)
    {
        // The given direction is invalid, so we just stop
        desired_direction_ = 0;
        desired_velocity_ = 0;
        current_direction_ = 0;
        current_velocity_ = 0;
        ROS_ERROR("Invalid turn direction on topic '%s'!", motion_command_topic_.c_str());
        return;
    }
    desired_direction_ = motion_cmd_msg->turn;
    desired_velocity_ = motion_cmd_msg->velocity;
    drive_mode_ = motion_cmd_msg->mode;
}

void MotionControlROS::loadParameters()
{
    std::string motion_command_topic, velocity_command_topic, robot_frame, odometry_frame;

    nh_.param<std::string>("motion_command_topic", motion_command_topic, "/motion_command");
    motion_command_topic_ = motion_command_topic;
    ROS_DEBUG("motion_command_topic: %s", motion_command_topic_.c_str());

    nh_.param<std::string>("velocity_command_topic", velocity_command_topic, "/ropod/cmd_vel");
    velocity_command_topic_ = velocity_command_topic;
    ROS_DEBUG("velocity_command_topic: %s", velocity_command_topic_.c_str());

    nh_.param<std::string>("robot_frame", robot_frame, "ropod/base_link");
    robot_frame_ = robot_frame;
    ROS_DEBUG("robot_frame: %s", robot_frame_.c_str());

    nh_.param<std::string>("odometry_frame", odometry_frame, "ropod/odom");
    odometry_frame_ = odometry_frame;
    ROS_DEBUG("odometry_frame: %s", odometry_frame_.c_str());
    
    double max_free_space;
    nh_.param<double>("max_free_space", max_free_space, 5.0);
    // TODO
    ROS_DEBUG("max_free_space: %f", max_free_space);

    double safety_decay;
    nh_.param<double>("safety_decay", safety_decay, 0.95);
    // TODO
    ROS_DEBUG("safety_decay: %f", safety_decay);

    double safety_weight;
    nh_.param<double>("safety_weight", safety_weight, 0.95);
    // TODO
    ROS_DEBUG("safety_weight: %f", safety_weight);

    double conformance_weight;
    nh_.param<double>("conformance_weight", conformance_weight, 1);
    // TODO
    ROS_DEBUG("conformance_weight: %f", conformance_weight);

    double continue_weight;
    nh_.param<double>("continue_weight", continue_weight, 1);
    // TODO
    ROS_DEBUG("continue_weight: %f", continue_weight);

    double escape_weight;
    nh_.param<double>("escape_weight", escape_weight, 1);
    // TODO
    ROS_DEBUG("escape_weight: %f", escape_weight);

    double max_velocity;
    nh_.param<double>("max_velocity", max_velocity, 1);
    // TODO
    ROS_DEBUG("max_velocity: %f", max_velocity);

    double desired_safety_threshold;
    nh_.param<double>("desired_safety_threshold", desired_safety_threshold, 1);
    // TODO
    ROS_DEBUG("desired_safety_threshold: %f", desired_safety_threshold);
}

bool MotionControlROS::motionControlSwitch(motion_control::Switch::Request  &req, motion_control::Switch::Response &res)
{
    is_enabled_ = req.enable;
    res.status = true;
    return true;
}