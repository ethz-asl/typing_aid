#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>

#include "LiftController.h"

int startGravityCompensation(bool &stopit)
{

    // Put robot into gravity compensation mode
    ROS_INFO("Starting gravity compensation mode");
    while (true && ros::ok())
    {
        if (stopit)
        {
            ROS_INFO("Stopping gravity compensation mode");
            break;
        }
        // ROS_INFO_STREAM("stopit: " << stopit);
        ros::Duration(0.1).sleep();
    }
    return 0;
}

void liftArm()
{
    ROS_INFO("Lifting arm");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lifting_listener");
    ros::NodeHandle n;

    bool lift_flag = false;
    LiftController lc(n, &lift_flag);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    while (ros::ok())
    {
        startGravityCompensation(lift_flag);
        liftArm();
        lift_flag = false;
    }

    return 0;
}
