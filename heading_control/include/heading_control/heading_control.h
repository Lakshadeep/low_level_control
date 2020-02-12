#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <cmath>

class HeadingControl
{

public:
    HeadingControl();
    ~HeadingControl();
    double computeDirection(double curr_heading, double desired_heading);
    void setProportionalGain(double gain);
    void setDerivativeGain(double gain);

private:
    double last_error_;
    double kp_, kd_;
    double wrapTo2Pi(double angle);
};

#endif
