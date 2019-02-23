#include <heading_control/heading_control.h>

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
    double error = curr_heading - desired_heading;
    double dir = kp_*(error)/(M_PI/2)  +  kd_*(error - last_error_)/(M_PI/2) ;

    if ( dir > 1.0)
        dir = 1.0;
    else if (dir < -1.0)
        dir = -1.0;

    last_error_ = error;
    return dir; 
}