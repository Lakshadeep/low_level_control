#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <cmath>

class HeadingControl
{

public:
  HeadingControl();
  ~HeadingControl();
  double computeDirection(double curr_heading, double desired_heading);
private:
};

#endif
