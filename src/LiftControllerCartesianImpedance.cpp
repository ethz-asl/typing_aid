//
// Created by fjulian on 23.07.20.
//

#include "LiftControllerCartesianImpedance.h"

LiftControllerCartesianImpedance::LiftControllerCartesianImpedance(ros::NodeHandle &n, franka::Robot *robot)
        : LiftController(n, robot), model(robot->loadModel()) {
    bool fixed_initial_position;
    n.param<bool>("predefined_initial_pose", fixed_initial_position, false);
    if (!n.getParam("impedance_control/time_limit", time_limit)) {
        ROS_ERROR("Parameter missing");
        throw;
    }

    setImpedanceBehavior(*robot);

    if (fixed_initial_position) {
        // First move the robot to a suitable joint configuration
        std::array<double, 7> lifted_joints = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
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
    franka::RobotState state = robot->readOnce();
    // ROS_INFO("Stiffness frame:");
    // printArray(state.EE_T_K.data(), state.EE_T_K.size());

    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    position_d = initial_transform.translation();
    orientation_d = initial_transform.linear();

    double translational_stiffness;
    if (!n.getParam("impedance_control/translational_stiffness", translational_stiffness)) {
        ROS_ERROR("Parameter missing");
        throw;
    }
    double rotational_stiffness;
    if (!n.getParam("impedance_control/rotational_stiffness", rotational_stiffness)) {
        ROS_ERROR("Parameter missing");
        throw;
    }
    stiffness.setZero();
    stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
    damping.setZero();
    damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                   Eigen::MatrixXd::Identity(3, 3);
    damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                       Eigen::MatrixXd::Identity(3, 3);
}

void LiftControllerCartesianImpedance::liftArm() {
    ROS_INFO("Lifting arm");
    double time = 0.0;
    lift_flag = false;
    try {
        // define callback for the torque control loop
        std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
                impedance_control_callback = [this, &time](const franka::RobotState &robot_state,
                                                    franka::Duration period) -> franka::Torques {
            time += period.toSec();

            // get state variables
            std::array<double, 7> coriolis_array = model.coriolis(robot_state);
            std::array<double, 42> jacobian_array =
                    model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
            // convert to Eigen
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
            Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
            Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
            Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
            Eigen::Vector3d position(transform.translation());
            Eigen::Quaterniond orientation(transform.linear());
            // compute error to desired equilibrium pose
            // position error
            Eigen::Matrix<double, 6, 1> error;
            error.head(3) << position - position_d;
            // orientation error
            // "difference" quaternion
            if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();
            }
            // "difference" quaternion
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            // Transform to base frame
            error.tail(3) << -transform.linear() * error.tail(3);
            // compute control
            Eigen::VectorXd tau_task(7), tau_d(7);
            // Spring damper system with damping ratio=1
            tau_task << jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
            tau_d << tau_task + coriolis;
            std::array<double, 7> tau_d_array{};
            Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
            return tau_d_array;
            // if (time < 1.0) {
            //     return tau_d_array;
            // } else {
            //     franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            //     return franka::MotionFinished(zero_torques); 
            // }
            
        };
        robot->control(impedance_control_callback);
    }
    catch (const franka::Exception &e) {
        ROS_ERROR_STREAM(e.what());
        throw e;
    }
}
