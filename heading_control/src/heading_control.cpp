#include <heading_control/heading_control.h>

HeadingControl::HeadingControl()
{
}

HeadingControl::~HeadingControl()
{
}

double HeadingControl::computeOperatorInputs(double curr_heading, double desired_heading)
{
    double ip = -(curr_heading - desired_heading)/1.57;

    if(ip > 1.0)
        ip = 1.0;
    else if (ip < -1.0)
        ip = -1.0;

    return ip; 
}