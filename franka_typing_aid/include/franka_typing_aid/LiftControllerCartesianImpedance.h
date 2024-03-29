//
// Created by fjulian on 23.07.20.
//

#ifndef TYPING_8_LIFTCONTROLLERCARTESIANIMPEDANCE_H
#define TYPING_8_LIFTCONTROLLERCARTESIANIMPEDANCE_H

#include <franka_typing_aid/LiftController.h>
#include <franka/model.h>
#include <Eigen/Dense>

class LiftControllerCartesianImpedance : public LiftController
{
public:
    LiftControllerCartesianImpedance(ros::NodeHandle &n, franka::Robot *robot);
    void liftArm() override;

private:
    Eigen::Vector3d position_d;
    Eigen::Quaterniond orientation_d;
    franka::Model model;
    Eigen::Matrix<double, 6, 6> stiffness;
    Eigen::Matrix<double, 6, 6> damping;
    double time_limit;
};

#endif //TYPING_8_LIFTCONTROLLERCARTESIANIMPEDANCE_H
