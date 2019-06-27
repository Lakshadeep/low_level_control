# Low level control
Contains ROS pacakges for motion planning and control.

## Heading control
Provides PD heading controller

#### Subscribed topics
* desired heading
* monitored heading

#### Published topics
* motion command

#### Advertised services
* switch for enabling/disabling heading controller

#### Launch
`roslaunch heading_control heading_control.launch`


## Motion control
Plans and executes the motion

#### Subscribed topics
* desired heading
* monitored heading

#### Published topics
* command velocities to the platform controller

#### Advertised services
* switch for enabling/disabling motion planner & controller
* configuring motion planner and controller params
* service for changing motion controller mode

#### Launch
`roslaunch motion_control motion_control.launch`


## All the low level control sub-compoenents can be run together using `low_level_control` meta-package as follows
`roslaunch low_level_control low_level_control.launch`





