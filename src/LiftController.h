#ifndef LIFT_CONTROLLER_H__
#define LIFT_CONTROLLER_H__

#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <franka/exception.h>

class LiftController {
public:
    LiftController(ros::NodeHandle &n, franka::Robot *robot_ptr);

    void buttonCallback(const std_msgs::Empty::ConstPtr &msg);

    virtual void liftArm() = 0;

    void startGravityCompensation();

protected:
    franka::Robot *robot;
private:
    ros::Subscriber sub;
    bool lift_flag;

};

#endif
