#!/usr/bin/env python
# coding=utf-8

import atexit

import rospy

import fsmstate as fsm
import utils
from torque_profile import cte_mov
from pid import pid
from pos_control import pos_mov
from imped_pos import Impedance_pos
from imped_vel import impedance_vel
from friction import Friction


if __name__ == "__main__":

    try:
        rospy.init_node("controller", anonymous=True)

        controller = input(
            "Choose the controller method  \n 1 = position controller \n 2 = torque controller \n 3 = pid controller \n 4 = impedance on pos controller \n 5 = impedance on vel controller \n 6 = friction controller \n"
        )
        if controller == 1:
            ctrl = pos_mov()
        elif controller == 2:
            ctrl = cte_mov()
        elif controller == 3:
            ctrl = pid()
        elif controller == 4:
            ctrl = Impedance_pos()
        elif controller == 5:
            ctrl = impedance_vel()
        elif controller == 6:
            ctrl = Friction()

        atexit.register(ctrl.stop)

        fsm.FSM_state().set_FSM_state(4)
        ctrl.run()

    except rospy.ROSInterruptException:
        utils.utils().stop_drive()