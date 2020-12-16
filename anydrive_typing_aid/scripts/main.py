#!/usr/bin/env python
# coding=utf-8

import rospy
import fsmstate as fsm
import init_mov
import utils

if __name__ == "__main__":

    try:
        rospy.init_node("controller", anonymous=True)
        #setting FSM_state
        #goes into ControlOp
        fsm.FSM_state().set_FSM_state(4)

        #initialization applying a constant torque
        init_mov().run(True)
        rospy.loginfo("init successful")

        #choose the controller

        #constant torque controller
        init_mov().run(False)

    except rospy.ROSInterruptException:
        utils().stop()
