#include <iostream>

#include <franka/robot.h>
#include <franka/exception.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "LiftControllerJointPosition.h"
#include "LiftControllerCartesianImpedance.h"


int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "lifting_listener");
    ros::NodeHandle n;

    // Get parameters
    std::string control_mode;
    n.param<std::string>("control_mode", control_mode, "position");

    // Initialize controller
    franka::Robot robot(argv[1]);
    LiftController *lc;
    if (control_mode == "position") {
        lc = new LiftControllerJointPosition(n, &robot);
    } else if (control_mode == "impedance") {
        lc = new LiftControllerCartesianImpedance(n, &robot);
    } else {
        ROS_ERROR("Invalid control mode defined");
        return -1;
    }

    ros::AsyncSpinner spinner(4); // Use 4 threads

    ROS_INFO("======================================================================");
    ROS_INFO("Setup complete. Once started, program can be stopped with ctrl-c.");
    ROS_INFO("Press any key to start...");
    std::cin.ignore();

    spinner.start();
    while (ros::ok()) {
        try {
            lc->startGravityCompensation();
            lc->liftArm();
        } catch (const franka::ControlException &e) {
            robot.automaticErrorRecovery();
            ROS_INFO("======================================================================");
            ROS_INFO_STREAM("Robot was stopped due to an error: " << e.what());
            ROS_INFO("Press any key to continue...");
            std::cin.ignore();
        }
    }

    return 0;
}
