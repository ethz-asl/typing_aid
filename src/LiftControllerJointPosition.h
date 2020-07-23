//
// Created by fjulian on 23.07.20.
//

#ifndef TYPING_8_LIFTCONTROLLERJOINTPOSITION_H
#define TYPING_8_LIFTCONTROLLERJOINTPOSITION_H

#include "LiftController.h"

class LiftControllerJointPosition : public LiftController {
public:
    LiftControllerJointPosition(ros::NodeHandle &n, franka::Robot *robot, bool fixed_initial_joints);
    void liftArm() override;
private:
    std::array<double, 7> lifted_joints{};
};


#endif //TYPING_8_LIFTCONTROLLERJOINTPOSITION_H
