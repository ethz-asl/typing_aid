#include "lift_controller.h"

LiftController::LiftController(ros::NodeHandle &n, franka::Robot *robot_ptr) : robot(robot_ptr) {
    lift_flag = false;
    sub = n.subscribe("lifting_trigger", 10, &LiftController::buttonCallback, this);
    try {
        std::array<double, 3> F_x_load = {{0.0, 0.0, 0.0}};
        std::array<double, 9> load_inertia = {{1.0, 0.0, 0.0,
                                                      0.0, 1.0, 0.0, 0.0, 0.0, 1.0}};
        robot->setLoad(0.7, F_x_load, load_inertia);
    } catch (const franka::Exception &e) {
        ROS_ERROR_STREAM(e.what());
        throw e;
    }
}

void LiftController::buttonCallback(const std_msgs::Empty::ConstPtr &msg) {
    ROS_INFO("Received trigger");
    lift_flag = true;
}

void LiftController::startGravityCompensation() {
    // Put robot into gravity compensation mode
    ROS_INFO("Starting gravity compensation mode");
    robot->control([this](const franka::RobotState &, franka::Duration) -> franka::Torques {
        franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        if (this->lift_flag || !ros::ok()) {
            ROS_INFO("Stopping gravity compensation mode");
            return franka::MotionFinished(zero_torques);
        }
        return zero_torques;
    });
}
