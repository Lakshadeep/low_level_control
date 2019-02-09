#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <vector>
#include <cmath>

class HeadingControl
{

public:
  HeadingControl();
  ~HeadingControl();
  double computeOperatorInputs(double curr_heading, double desired_heading);
private:
};

#endif
