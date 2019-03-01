#include "motion_control/motion_control_ros.h"
#include <ros/console.h>

int main(int argc, char **argv)
{

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG("Starting motion control node");

    ros::init(argc, argv, "motion_control_node");
    ros::NodeHandle nh("~");
    MotionControlROS motion_control_ros(nh);

    double frequency;
    nh.param<double>("frequency", frequency, 10);
    ROS_DEBUG("Frequency set to %0.1f Hz", frequency);
    ros::Rate rate(frequency);

    while (ros::ok())
    {
        motion_control_ros.executeCommand();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

