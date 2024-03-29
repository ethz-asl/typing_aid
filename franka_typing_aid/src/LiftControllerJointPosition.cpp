//
// Created by fjulian on 23.07.20.
//

#include <franka_typing_aid/LiftControllerJointPosition.h>

LiftControllerJointPosition::LiftControllerJointPosition(ros::NodeHandle &n, franka::Robot *robot)
    : LiftController(n, robot)
{
    bool fixed_initial_joints;
    n.param<bool>("predefined_initial_pose", fixed_initial_joints, false);
    n.param<float>("position_control/speed", liftingSpeed, 0.25);

    setStrongBehavior(*robot);

    franka::RobotState state = robot->readOnce();
    ROS_INFO("Stiffness frame:");
    printArray(state.EE_T_K.data(), state.EE_T_K.size());

    if (fixed_initial_joints)
    {
        // First move the robot to a suitable joint configuration
        lifted_joints = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        float speed;
        n.param<float>("initial_position_speed", speed, 0.4);
        MotionGenerator motion_generator(speed, lifted_joints);
        ROS_INFO_STREAM("WARNING: This example will move the robot! "
                        << "Please make sure to have the user stop button at hand!" << std::endl
                        << "Press Enter to move to initial position...");
        std::cin.ignore();
        robot->control(motion_generator);
        ROS_INFO("Finished moving to initial joint configuration.");
    }
    else
    {
        lifted_joints = state.q;
    }
}

void LiftControllerJointPosition::liftArm()
{
    ROS_INFO("Lifting arm");
    lift_flag = false;

    // Two options:
    // 1) Lift arm to pre-defined position
    // 2) Lift arm by relative distance, from current position (not implemented yet)

    try
    {
        MotionGenerator motion_generator(liftingSpeed, lifted_joints);
        robot->control(motion_generator);
    }
    catch (const franka::Exception &e)
    {
        ROS_ERROR_STREAM(e.what());
        throw e;
    }
}
