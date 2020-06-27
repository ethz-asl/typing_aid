#include <iostream>

#include <franka/robot.h>
#include <franka/exception.h>

#include <ros/ros.h>
#include <ros/console.h>

#include "examples_common.h"
#include "lift_controller.h"

std::array<double, 7> setupFrankaArm(franka::Robot &robot, bool fixed_initial_pos)
{
    std::array<double, 7> ret;
    try
    {
        setStrongBehavior(robot);
        std::array<double, 3> F_x_load = {{0.0, 0.0, 0.0}};
        std::array<double, 9> load_inertia = {{1.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};

        robot.setLoad(0.7, F_x_load, load_inertia);

        franka::RobotState state = robot.readOnce();
        ROS_INFO("Stiffness frame:");
        printArray(state.EE_T_K.data(), state.EE_T_K.size());

        if (fixed_initial_pos)
        {
            // First move the robot to a suitable joint configuration
            ret = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            MotionGenerator motion_generator(0.5, ret);
            ROS_INFO_STREAM("WARNING: This example will move the robot! "
                            << "Please make sure to have the user stop button at hand!" << std::endl
                            << "Press Enter to move to initial position...");
            std::cin.ignore();
            robot.control(motion_generator);
            ROS_INFO("Finished moving to initial joint configuration.");
        }
        else
        {
            ret = state.q;
        }
    }
    catch (const franka::Exception &e)
    {
        ROS_ERROR_STREAM(e.what());
        throw e;
    }
    return ret;
}

void startGravityCompensation(franka::Robot &robot, bool &stopit)
{
    // try
    // {
        // Put robot into gravity compensation mode
        ROS_INFO("Starting gravity compensation mode");
        robot.control([&stopit](const franka::RobotState &, franka::Duration) -> franka::Torques {
            franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            if (stopit || !ros::ok())
            {
                ROS_INFO("Stopping gravity compensation mode");
                return franka::MotionFinished(zero_torques);
            }
            return zero_torques;
        });
    // }
    // catch (const franka::Exception &e)
    // {
    //     ROS_ERROR_STREAM(e.what());
    //     throw e;
    // }
}

void liftArm(franka::Robot &robot, std::array<double, 7> lifted_joints)
{
    ROS_INFO("Lifting arm");

    // Two options:
    // 1) Lift arm to pre-defined position
    // 2) Lift arm by relative distance, from current position (not implemented yet)

    try
    {
        MotionGenerator motion_generator(0.25, lifted_joints);
        robot.control(motion_generator);
    }
    catch (const franka::Exception &e)
    {
        ROS_ERROR_STREAM(e.what());
        throw e;
    }
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "lifting_listener");
    ros::NodeHandle n;

    bool lift_flag = false;
    LiftController lc(n, &lift_flag);

    franka::Robot robot(argv[1]);
    bool fixed_lifted_joints = false; // TODO move to config file
    std::array<double, 7> lifted_joints = setupFrankaArm(robot, fixed_lifted_joints);

    ros::AsyncSpinner spinner(4); // Use 4 threads

    ROS_INFO("======================================================================");
    ROS_INFO("Setup complete. Once started, program can be stopped with ctrl-c.");
    ROS_INFO("Press any key to start...");
    std::cin.ignore();

    spinner.start();
    while (ros::ok())
    {
        try {
            lift_flag = false;
            startGravityCompensation(robot, lift_flag);
            liftArm(robot, lifted_joints);
        } catch (const franka::ControlException& e) {
            robot.automaticErrorRecovery();
            ROS_INFO("======================================================================");
            ROS_INFO_STREAM("Robot was stopped due to an error: " << e.what());
            ROS_INFO("Press any key to continue...");
            std::cin.ignore();
        }
    }

    return 0;
}
