#ifndef HEADING_CONTROL_ROS_H
#define HEADING_CONTROL_ROS_H

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "heading_control/heading_control.h"
#include "heading_control/Switch.h"
#include "nav2d_operator/cmd.h"


class HeadingControlROS
{

public:
  // Constructor / destructor
  HeadingControlROS(ros::NodeHandle&);
  ~HeadingControlROS();
  // Running
  void run();

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber desired_heading_subscriber_, desired_velocity_subscriber_, monitored_heading_subscriber_ ;
  ros::ServiceServer heading_control_switch_service_;
  ros::Publisher operator_cmd_publisher_;

  // Parameters
  std::string desired_heading_topic_, desired_velocity_topic_, monitored_heading_topic_ ;

  // robot heading monitoring
  HeadingControl heading_control_;
  bool is_enabled_; 
  double monitored_heading_, desired_heading_, desired_velocity_;
  double heading_control_velocity_;


  // Members
  void loadParameters();
  void desiredHeadingCallback(const std_msgs::Float32ConstPtr& desired_heading_msg);
  void desiredVelocityCallback(const std_msgs::Float32ConstPtr& desired_velocity_msg);
  void monitoredHeadingCallback(const std_msgs::Float32ConstPtr& monitored_heading_msg);
  bool headingControlSwitch(heading_control::Switch::Request  &req, heading_control::Switch::Response &res);

};

#endif
