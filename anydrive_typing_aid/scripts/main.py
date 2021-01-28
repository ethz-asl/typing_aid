#!/usr/bin/env python
# coding=utf-8

import atexit

import rospy

import fsmstate as fsm
import utils
import torque_profile as c
import pid as p
import pos_control as pos
import imped_pos as i_pos
import imped_vel as i_vel
from friction import Friction


if __name__ == "__main__":

    try:
        rospy.init_node("controller", anonymous=True)

        controller = input(
            "Choose the controller method  \n 1 = position controller \n 2 = torque controller \n 3 = pid controller \n 4 = impedance on pos controller \n 5 = impedance on vel controller \n 6 = friction controller \n"
        )
        if controller == 1:
            ctrl = pos.pos_mov()
        elif controller == 2:
            ctrl = c.cte_mov()
        elif controller == 3:
            ctrl = p.pid()
        elif controller == 4:
            ctrl = i_pos.impedance()
        elif controller == 5:
            ctrl = i_vel.impedance_vel()
        elif controller == 6:
            ctrl = Friction()
        # setting FSM_state
        # goes into ControlOp

        atexit.register(ctrl.stop)

        fsm.FSM_state().set_FSM_state(4)
        ctrl.run()

    except rospy.ROSInterruptException:
        utils.utils().stop()