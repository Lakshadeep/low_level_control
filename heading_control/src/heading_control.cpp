#include <heading_control/heading_control.h>
#include <ros/ros.h>

HeadingControl::HeadingControl():last_error_(0)
{
}

HeadingControl::~HeadingControl()
{
}

void HeadingControl::setProportionalGain(double gain)
{ 
    kp_ = gain;
}

void HeadingControl::setDerivativeGain(double gain)
{ 
    kd_ = gain;
}

double HeadingControl::computeDirection(double curr_heading, double desired_heading)
{
    //choose the smaller angle for rotation
    double error = 0.0;
    if (curr_heading < 0) 
        curr_heading = (2 * M_PI) - fabs(curr_heading);

    if (desired_heading < 0) 
        desired_heading = (2 * M_PI) - fabs(desired_heading);

    error = (desired_heading - curr_heading);
    
    if (error > M_PI) 
        error = error - (2 * M_PI);
    else if (error < -M_PI) 
        error = (2 * M_PI) + error;

    double dir =  - (kp_*(error)/(M_PI/2)  +  kd_*(error - last_error_)/(M_PI/2)) ;

    if ( dir > 1.0)
        dir = 1.0;
    else if (dir < -1.0)
        dir = -1.0;

    last_error_ = error;
    return dir; 
}

double HeadingControl::wrapTo2Pi(double angle)
{
   return fmod(angle + 2 * M_PI, 2 * M_PI);
}