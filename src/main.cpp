#include <iostream>

#include <franka/robot.h>
#include <franka/exception.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "examples_common.h"
#include "lift_controller.h"

int setupFrankaArm(franka::Robot &robot)
{
    try
    {
        setDefaultBehavior(robot);

        // First move the robot to a suitable joint configuration
        std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        ROS_INFO_STREAM("WARNING: This example will move the robot! "
                        << "Please make sure to have the user stop button at hand!" << std::endl
                        << "Press Enter to continue...");
        std::cin.ignore();
        robot.control(motion_generator);
        ROS_INFO("Finished moving to initial joint configuration.");
    }
    catch (const franka::Exception &e)
    {
        ROS_INFO_STREAM(e.what());
        return -1;
    }
    return 0;
}

int startGravityCompensation(franka::Robot &robot, bool &stopit)
{
    try
    {
        // Put robot into gravity compensation mode
        robot.control([&stopit](const franka::RobotState &, franka::Duration) -> franka::Torques {
            franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            if (stopit)
            {
                ROS_INFO("Stopping gravity compensation mode");
                return franka::MotionFinished(zero_torques);
            }
            return zero_torques;
        });
    }
    catch (const franka::Exception &e)
    {
        ROS_INFO_STREAM(e.what());
        return -1;
    }
    return 0;
}

void liftArm(franka::Robot &robot)
{
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "lifting-listener");
    ros::NodeHandle n;

    bool lift_flag = false;
    LiftController lc(n, &lift_flag);

    franka::Robot robot(argv[1]);
    setupFrankaArm(robot);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();

    while (ros::ok())
    {
        startGravityCompensation(robot, lift_flag);
    }

    return 0;
}
