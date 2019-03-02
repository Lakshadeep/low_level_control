#ifndef MOTION_CONTROL_ROS_H
#define MOTION_CONTROL_ROS_H

#define LUT_RESOLUTION 100

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/GridCells.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <motion_control/Command.h>
#include <motion_control/Switch.h>
#include <motion_control/Params.h>
#include <motion_control/DriveMode.h>

#include <motion_control/motion_control.h>


class MotionControlROS
{

public:
    // Constructor / destructor
    MotionControlROS(ros::NodeHandle&);
    ~MotionControlROS();

    /*
    @brief Generates and sends Twist-Message to Robot
    Executes motion controller
     */
    void executeCommand();

private:
    // ROS sub/pub
    ros::NodeHandle nh_;
    ros::Subscriber motion_command_subscriber_;
    ros::ServiceServer motion_control_switch_service_;
    ros::ServiceServer motion_control_params_service_;
    ros::ServiceServer motion_control_drive_mode_service_;
    ros::Publisher velocity_command_publisher_;
    ros::Publisher trajectory_publisher_;
    ros::Publisher plan_publisher_;

    // ROS tf
    tf::TransformListener tf_listener_;

    // costmap params
    costmap_2d::Costmap2DROS* local_map;
    costmap_2d::Costmap2D* costmap;
    double raster_size_;

    // Parameters
    std::string motion_command_topic_, velocity_command_topic_, robot_frame_, odometry_frame_;

    double max_free_space_, safety_decay_, safety_weight_, conformance_weight_, continue_weight_, escape_weight_, 
           max_velocity_, desired_safety_threshold_;
    unsigned int recovery_steps_;

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

    // other internal vars
    sensor_msgs::PointCloud* trajectory_table[(LUT_RESOLUTION * 4) + 2];

    // Members
    void loadParameters();
    void motionCommandCallback(const motion_control::CommandConstPtr& motion_cmd_msg);
    bool motionControlSwitch(motion_control::Switch::Request  &req, motion_control::Switch::Response &res);
    bool setDriveMode(motion_control::DriveMode::Request  &req, motion_control::DriveMode::Response &res);
    bool reConfigureParams(motion_control::Params::Request  &req, motion_control::Params::Response &res);

    /*
    @brief Initializes look-up tables
    This calculates the trajectories of all possible direction commands.
    Must be called once before the Operator is used
    */
    void initTrajTable();

    /*
    @brief Calculates the distance the robot can move following the given trajectory
    @param cloud PointCloud defining a trajectory
    @return Number of free cells
    */
    int calculateFreeSpace(sensor_msgs::PointCloud* cloud);
    

    /*
    @brief Calculate the action value of a given command
    @param direction How to move the robot
    @param velocity Only used to distinguish forward and backward motion
    @param debug Publish result of evaluation functions on debug topic
    @return Weighted sum of all evaluation functions
    The given action is rated by 4 different evaluation functions:
    Free Space: How far can the robot move following this command
    Safety: How close will it get near obstacles
    Conformance: How good does it follow the commanded direction
    Continuity: (experimental!) How does it conform with the last issued command
     */
    double evaluateAction(double direction, double velocity, bool debug);

    /**
    @brief Get the trajectory defined by the given movement command
    @param direction How to move the robot
    @param velocity Only used to distinguish forward and backward motion
    @return A pointer to the PointCloud defined in the robot coordinate frame
     */
    sensor_msgs::PointCloud* getPointCloud(double direction, double velocity);
    double findBestDirection();

};

#endif
