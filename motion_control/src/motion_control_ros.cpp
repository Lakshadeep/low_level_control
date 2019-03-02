#include "motion_control/motion_control_ros.h"

MotionControlROS::MotionControlROS(ros::NodeHandle& nh): nh_(nh), is_enabled_(false), tf_listener_(ros::Duration(0.5))
{
    loadParameters();

    // subscribers, publishers and services
    motion_command_subscriber_ = nh_.subscribe(motion_command_topic_, 1, &MotionControlROS::motionCommandCallback, this);
    motion_control_switch_service_ = nh.advertiseService("/motion_control_switch", &MotionControlROS::motionControlSwitch, this);
    motion_control_params_service_ = nh.advertiseService("/motion_control_params", &MotionControlROS::reConfigureParams, this);
    motion_control_drive_mode_service_ = nh.advertiseService("/motion_control_drive_mode", &MotionControlROS::setDriveMode, this);
    velocity_command_publisher_ = nh_.advertise<geometry_msgs::Twist>(velocity_command_topic_, 1);
    trajectory_publisher_ = nh_.advertise<nav_msgs::GridCells>("/route", 1);
    plan_publisher_ = nh_.advertise<nav_msgs::GridCells>("/plan", 1);

    // Create the local costmap
    local_map = new costmap_2d::Costmap2DROS(std::string("local_map"), tf_listener_);
    raster_size_ = local_map->getCostmap()->getResolution();

    // Apply tf_prefix to all used frame-id's
    robot_frame_ = tf_listener_.resolve(robot_frame_);
    odometry_frame_ = tf_listener_.resolve(odometry_frame_);

    // Initialize the lookup table for the driving directions
    ROS_DEBUG("Initializing LUT...");
    initTrajTable();
    
    // Set internal parameters
    desired_direction_ = 0;
    desired_velocity_ = 0;
    current_direction_ = 0;
    current_velocity_ = 0;
    drive_mode_ = 0;
    recovery_steps_ = 0;

    ROS_DEBUG("Initialization complete");
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
    max_free_space_ = max_free_space;
    ROS_DEBUG("max_free_space: %f", max_free_space);

    double safety_decay;
    nh_.param<double>("safety_decay", safety_decay, 0.95);
    safety_decay_ = safety_decay;
    ROS_DEBUG("safety_decay: %f", safety_decay);

    double safety_weight;
    nh_.param<double>("safety_weight", safety_weight, 0.95);
    safety_weight_ = safety_weight;
    ROS_DEBUG("safety_weight: %f", safety_weight);

    double conformance_weight;
    nh_.param<double>("conformance_weight", conformance_weight, 1);
    conformance_weight_ = conformance_weight;
    ROS_DEBUG("conformance_weight: %f", conformance_weight);

    double continue_weight;
    nh_.param<double>("continue_weight", continue_weight, 1);
    continue_weight_ = continue_weight;
    ROS_DEBUG("continue_weight: %f", continue_weight);

    double escape_weight;
    nh_.param<double>("escape_weight", escape_weight, 1);
    escape_weight_ = escape_weight;
    ROS_DEBUG("escape_weight: %f", escape_weight);

    double max_velocity;
    nh_.param<double>("max_velocity", max_velocity, 1);
    max_velocity_ = max_velocity;
    ROS_DEBUG("max_velocity: %f", max_velocity);

    double desired_safety_threshold;
    nh_.param<double>("desired_safety_threshold", desired_safety_threshold, 0.5);
    desired_safety_threshold_ = desired_safety_threshold;
    ROS_DEBUG("desired_safety_threshold: %f", desired_safety_threshold);
}

bool MotionControlROS::motionControlSwitch(motion_control::Switch::Request  &req, motion_control::Switch::Response &res)
{
    is_enabled_ = req.enable;
    res.status = true;
    return true;
}

bool MotionControlROS::setDriveMode(motion_control::DriveMode::Request  &req, motion_control::DriveMode::Response &res)
{
    drive_mode_ = req.drive_mode;
    res.drive_mode = drive_mode_;
    return true;
}

bool MotionControlROS::reConfigureParams(motion_control::Params::Request  &req, motion_control::Params::Response &res)
{
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::DoubleParameter double_param;
    dynamic_reconfigure::Config conf;

    double_param.name = "inflation_radius";
    double_param.value = req.inflation_radius;
    conf.doubles.push_back(double_param);

    srv_req.config = conf;

    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();

    if(ros::service::call("/motion_control/local_map/inflation_layer/set_parameters", srv_req, srv_resp))
    {    
        res.inflation_radius = req.inflation_radius;
        spinner.stop();
        return true;
    }

    spinner.stop();
    return false;
} 

void MotionControlROS::initTrajTable()
{
    for (int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
    {
        trajectory_table[i] = NULL;
    }
    for (int i = 1; i < LUT_RESOLUTION; i++)
    {
        double tw = -M_PI * i / LUT_RESOLUTION;
        double tx = cos(tw) + 1;
        double ty = -sin(tw);
        double tr = ((tx * tx) + (ty * ty)) / (ty + ty);
        std::vector<geometry_msgs::Point32> points;
        double alpha = 0;
        while (alpha < M_PI)
        {
            double x = tr * sin(alpha);
            double y = tr * (1.0 - cos(alpha));
            geometry_msgs::Point32 p;
            p.x = x;
            p.y = y;
            p.z = 0;
            points.push_back(p);
            alpha += raster_size_ / tr;
        }
        // Add the PointCloud to the LUT
        // Circle in forward-left direction
        sensor_msgs::PointCloud* flcloud = new sensor_msgs::PointCloud();
        flcloud->header.stamp = ros::Time(0);
        flcloud->header.frame_id = robot_frame_;
        flcloud->points.resize(points.size());

        // Circle in forward-right direction
        sensor_msgs::PointCloud* frcloud = new sensor_msgs::PointCloud();
        frcloud->header.stamp = ros::Time(0);
        frcloud->header.frame_id = robot_frame_;
        frcloud->points.resize(points.size());

        // Circle in backward-left direction
        sensor_msgs::PointCloud* blcloud = new sensor_msgs::PointCloud();
        blcloud->header.stamp = ros::Time(0);
        blcloud->header.frame_id = robot_frame_;
        blcloud->points.resize(points.size());

        // Circle in backward-right direction
        sensor_msgs::PointCloud* brcloud = new sensor_msgs::PointCloud();
        brcloud->header.stamp = ros::Time(0);
        brcloud->header.frame_id = robot_frame_;
        brcloud->points.resize(points.size());

        for (unsigned int j = 0; j < points.size(); j++)
        {
            flcloud->points[j] = points[j];
            frcloud->points[j] = points[j];
            blcloud->points[j] = points[j];
            brcloud->points[j] = points[j];

            frcloud->points[j].y *= -1;
            blcloud->points[j].x *= -1;
            brcloud->points[j].x *= -1;
            brcloud->points[j].y *= -1;
        }
        trajectory_table[LUT_RESOLUTION - i] = flcloud;
        trajectory_table[LUT_RESOLUTION + i] = frcloud;
        trajectory_table[(3 * LUT_RESOLUTION + 1) - i] = blcloud;
        trajectory_table[(3 * LUT_RESOLUTION + 1) + i] = brcloud;
    }

    // Add First and Last LUT-element
    geometry_msgs::Point32 p;
    p.x = 0;
    p.y = 0;
    p.z = 0;

    sensor_msgs::PointCloud* turn = new sensor_msgs::PointCloud();
    turn->header.stamp = ros::Time(0);
    turn->header.frame_id = robot_frame_;
    turn->points.resize(1);
    turn->points[0] = p;

    int straight_len = 5.0 / raster_size_;

    sensor_msgs::PointCloud* fscloud = new sensor_msgs::PointCloud();
    fscloud->header.stamp = ros::Time(0);
    fscloud->header.frame_id = robot_frame_;
    fscloud->points.resize(straight_len);

    sensor_msgs::PointCloud* bscloud = new sensor_msgs::PointCloud();
    bscloud->header.stamp = ros::Time(0);
    bscloud->header.frame_id = robot_frame_;
    bscloud->points.resize(straight_len);

    for (int i = 0; i < straight_len; i++)
    {
        fscloud->points[i] = p;
        bscloud->points[i] = p;
        bscloud->points[i].x *= -1;
        p.x += raster_size_;
    }

    trajectory_table[LUT_RESOLUTION] = fscloud;
    trajectory_table[LUT_RESOLUTION * 3 + 1] = bscloud;

    trajectory_table[0] = turn;
    trajectory_table[LUT_RESOLUTION * 2] = turn;
    trajectory_table[LUT_RESOLUTION * 2 + 1] = turn;
    trajectory_table[LUT_RESOLUTION * 4 + 1] = turn;

    for (int i = 0; i < (LUT_RESOLUTION * 4) + 2; i++)
    {
        if (!trajectory_table[i])
        {
            ROS_ERROR("Table entry %d has not been initialized!", i);
        }
    }
}


int MotionControlROS::calculateFreeSpace(sensor_msgs::PointCloud* cloud)
{
    unsigned int mx, my;
    int length = cloud->points.size();
    int freeSpace = 0;
    for (int i = 0; i < length; i++)
    {
        if (costmap->worldToMap(cloud->points[i].x, cloud->points[i].y, mx, my))
        {
            if (costmap->getCost(mx, my) < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                freeSpace++;
            } else
            {
                break;
            }
        } else
        {
            break;
        }
    }
    return freeSpace;
}

sensor_msgs::PointCloud* MotionControlROS::getPointCloud(double direction, double velocity)
{
    if (direction < -1) direction = -1;
    if (direction > 1) direction = 1;
    int offset = (velocity >= 0) ? LUT_RESOLUTION : 3 * LUT_RESOLUTION + 1;
    int table_index = (direction * LUT_RESOLUTION) + offset;
    return trajectory_table[table_index];
}


double MotionControlROS::evaluateAction(double direction, double velocity, bool debug)
{
    sensor_msgs::PointCloud* originalCloud = getPointCloud(direction, velocity);
    sensor_msgs::PointCloud transformedCloud;
    try
    {
        tf_listener_.transformPointCloud(odometry_frame_, *originalCloud, transformedCloud);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return 1;
    }

    double value_safety = 0.0;      // How safe is it to move in that direction?
    double value_escape = 0.0;      // How much does the safety improve?
    double value_conformance = 0.0; // How conform is it with the desired direction?
    double value_continue = 0.0;    // How conform is it with the previous command?

    double decay = 1.0;
    double safe_max = 0.0;
    double cost_max = 0.0;
    double cost_start = 1.0;

    // Calculate safety and escape value
    int length = transformedCloud.points.size();
    for (int i = 0; i < length; i++)
    {
        unsigned int mx, my;
        double cell_cost;
        if (costmap->worldToMap(transformedCloud.points[i].x, transformedCloud.points[i].y, mx, my))
        {
            cell_cost = (double)costmap->getCost(mx, my) / costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
            if (cell_cost >= 1.0)
            {
                // Trajectory hit an obstacle
                break;
            }
        }
        if (i == 0)
            cost_start = cell_cost;
        double cost = cell_cost * decay;
        double safe = (cost_start - cell_cost) * decay * 2.0;

        if (cost > cost_max) cost_max = cost;
        if (safe > safe_max) safe_max = safe;

        decay *= safety_decay_;
    }

    double action_value = 0.0;
    double norm_factor = 0.0;

    // Add safety value
    value_safety = 1.0 - cost_max;
    action_value += value_safety * safety_weight_;
    norm_factor += safety_weight_;

    // Add escape value
    value_escape = safe_max;
    action_value += value_escape * escape_weight_;
    norm_factor += escape_weight_;

    if (recovery_steps_ == 0)
    {
        // Calculate continuety value
        value_continue = (current_direction_ - direction) * 0.5;
        value_continue = 1.0 - (value_continue * value_continue);

        // Calculate conformance value
        double corr = (desired_direction_ - direction) * M_PI;
        value_conformance = 0.5 * cos(corr) + 0.5;

        // Add both to action value
        action_value += value_conformance * conformance_weight_;
        action_value += value_continue * continue_weight_;
        norm_factor += conformance_weight_ + continue_weight_;
    }

    action_value /= norm_factor;

    return action_value;
}

double MotionControlROS::findBestDirection()
{
    double best_dir = -1.0;
    double best_value = 0.0;
    double step = 0.01;
    double dir = -1.0;

    while (dir <= 1.0)
    {
        double value = evaluateAction(dir, desired_velocity_, false);
        if (value > best_value)
        {
            best_dir = dir;
            best_value = value;
        }
        dir += step;
    }
    return best_dir;
}

void MotionControlROS::executeCommand()
{
    if(!is_enabled_)
        return;

    // 1. Get a copy of the costmap to work on.
    costmap = local_map->getCostmap();
    boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));
    double best_direction, d, desired_direction_val;

    // 2. Set velocity and direction depending on drive mode
    switch (drive_mode_)
    {
    case 0:
        best_direction = findBestDirection();
        desired_direction_val = evaluateAction(desired_direction_, current_velocity_, false);
        if (desired_direction_val >= desired_safety_threshold_) best_direction = desired_direction_;
        d = best_direction - current_direction_;
        // if (d < -0.2) d = -0.2;
        // if (d > 0.2) d = 0.2;
        current_direction_ += d;
        current_velocity_ = desired_velocity_;
        break;
    case 1:
        current_direction_ = desired_direction_;
        current_velocity_ = desired_velocity_;
        break;
    default:
        ROS_ERROR("Invalid drive mode!");
        current_velocity_ = 0.0;
    }


    sensor_msgs::PointCloud* originalCloud = getPointCloud(current_direction_, desired_velocity_);
    sensor_msgs::PointCloud transformedCloud;

    try
    {
        tf_listener_.transformPointCloud(odometry_frame_, *originalCloud, transformedCloud);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        return;
    }

    // Determine maximum linear velocity
    int free_cells = calculateFreeSpace(&transformedCloud);
    double free_space = raster_size_ * free_cells;

    double safe_velocity_ = (free_space / max_free_space_) + 0.05;
    if (free_cells == transformedCloud.points.size() && safe_velocity_ < 0.5)
        safe_velocity_ = 0.5;

    if (free_space < 0.5 && free_cells < transformedCloud.points.size())
        safe_velocity_ = 0;

    if (safe_velocity_ > max_velocity_)
        safe_velocity_ = max_velocity_;

    // Check whether the robot is stuck
    if (recovery_steps_ > 0) recovery_steps_--;
    if (safe_velocity_ < 0.1)
    {
        if (drive_mode_ == 0)
        {
            recovery_steps_ = 30; // Recover for 3 seconds
            ROS_WARN_THROTTLE(1, "Robot is stuck! Trying to recover...");
        } else
        {
            current_velocity_ = 0;
            ROS_WARN_THROTTLE(1, "Robot cannot move further in this direction!");
        }
    }

    // Publish route via ROS (mainly for debugging)
    if (false)
    {
        nav_msgs::GridCells route_msg;
        route_msg.header.frame_id = odometry_frame_;
        route_msg.header.stamp = ros::Time::now();

        route_msg.cell_width = costmap->getResolution();
        route_msg.cell_height = costmap->getResolution();

        route_msg.cells.resize(free_cells);
        for (int i = 0; i < free_cells; i++)
        {
            route_msg.cells[i].x = transformedCloud.points[i].x;
            route_msg.cells[i].y = transformedCloud.points[i].y;
            route_msg.cells[i].z = transformedCloud.points[i].z;
        }
        trajectory_publisher_.publish(route_msg);

        // Publish plan via ROS (mainly for debugging)
        sensor_msgs::PointCloud* originalPlanCloud = getPointCloud(desired_direction_, desired_velocity_);
        sensor_msgs::PointCloud transformedPlanCloud;

        try
        {
            tf_listener_.transformPointCloud(odometry_frame_, *originalPlanCloud, transformedPlanCloud);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
        nav_msgs::GridCells plan_msg;
        plan_msg.header = route_msg.header;

        plan_msg.cell_width = costmap->getResolution();
        plan_msg.cell_height = costmap->getResolution();

        int free_space_plan = calculateFreeSpace(&transformedPlanCloud);
        plan_msg.cells.resize(free_space_plan);
        for (int i = 0; i < free_space_plan; i++)
        {
            plan_msg.cells[i].x = transformedPlanCloud.points[i].x;
            plan_msg.cells[i].y = transformedPlanCloud.points[i].y;
            plan_msg.cells[i].z = transformedPlanCloud.points[i].z;
        }
        plan_publisher_.publish(plan_msg);
    }

    // Publish result via Twist-Message
    geometry_msgs::Twist velocity_command_msg_;
    double velocity = current_velocity_;
    if (current_direction_ == 0)
    {
        if (velocity > safe_velocity_)
        {
            ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safe_velocity_);
            velocity = safe_velocity_;
        } else if (velocity < -safe_velocity_)
        {
            ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safe_velocity_);
            velocity = -safe_velocity_;
        }
        velocity_command_msg_.linear.x = velocity;
        velocity_command_msg_.angular.z = 0;
    } else if (current_direction_ == -1 || current_direction_ == 1)
    {
        velocity_command_msg_.linear.x = 0;
        velocity_command_msg_.angular.z = -1.0 * current_direction_ * velocity;
    } else
    {
        double x = sin(current_direction_ * M_PI);
        double y = (cos(current_direction_ * M_PI) + 1);
        double r = ((x * x) + (y * y)) / (2 * x);
        double abs_r = (r > 0) ? r : -r;
        velocity /= (1 + (1.0 / abs_r));
        if (velocity > safe_velocity_)
        {
            ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, safe_velocity_);
            velocity = safe_velocity_;
        } else if (velocity < -safe_velocity_)
        {
            ROS_DEBUG("Desired velocity of %.2f is limited to %.2f", velocity, -safe_velocity_);
            velocity = -safe_velocity_;
        }

        velocity_command_msg_.linear.x = velocity;
        velocity_command_msg_.angular.z = -1.0 / r * velocity_command_msg_.linear.x;
    }
    velocity_command_publisher_.publish(velocity_command_msg_);
}