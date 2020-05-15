#ifndef LIFT_CONTROLLER_H__
#define LIFT_CONTROLLER_H__

#include "ros/ros.h"
#include <std_msgs/Empty.h>

class LiftController
{
public:
    LiftController(ros::NodeHandle &n, bool *lift_flag);
    void buttonCallback(const std_msgs::Empty::ConstPtr &msg);

private:
    ros::Subscriber sub;
    bool *lift_flag_ptr;
};

#endif
