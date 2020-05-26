#include "lift_controller.h"

LiftController::LiftController(ros::NodeHandle &n, bool *lift_flag)
{
    lift_flag_ptr = lift_flag;
    sub = n.subscribe("lifting_trigger", 10, &LiftController::buttonCallback, this);
}

void LiftController::buttonCallback(const std_msgs::Empty::ConstPtr &msg)
{
    ROS_INFO("Received trigger");
    *lift_flag_ptr = true;
}
