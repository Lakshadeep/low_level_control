# Low level control
Contains ROS pacakges for low level controllers such as heading, distance etc.

## Heading control
Provides proportional heading controller

#### Subscribed topics
* desired heading
* monitored heading

#### Published topics
* cmd to nav2d operator

#### Advertised service
* switch for enabling/disabling heading controller

#### Launch
`roslaunch heading_control heading_control.launch`

