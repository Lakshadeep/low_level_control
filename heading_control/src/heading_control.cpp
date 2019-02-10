#include <heading_control/heading_control.h>

HeadingControl::HeadingControl()
{
}

HeadingControl::~HeadingControl()
{
}

double HeadingControl::computeDirection(double curr_heading, double desired_heading)
{
    double dir = (curr_heading - desired_heading)/(M_PI/2);

    if ( dir > 1.0)
        dir = 1.0;
    else if (dir < -1.0)
        dir = -1.0;

    return dir; 
}