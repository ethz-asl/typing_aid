#!/usr/bin/env python
# coding=utf-8

import rospy
import fsmstate as fsm
import init_mov
import utils

position = {
    "up_position": -4,
    "down_position": -2,
    "up_limit": -5.5,
    "down_limit": 0,
    "v_max": 5,
    "t_min":0.3,
    "t_max":1.5
}

if __name__ == "__main__":

    try:
        rospy.init_node("controller", anonymous=True)
        #setting FSM_state
        #goes into ControlOp
        fsm.FSM_state().set_FSM_state(4)

        #initialization
        init_mov().run(True,position)
        rospy.loginfo("init successful")

        #choose the controller

        #constant torque controller
        init_mov().run(False,position)

    except rospy.ROSInterruptException:
        utils().stop()
