#!/usr/bin/env python
# coding=utf-8

import rospy

import fsmstate as fsm
import InitMove
import utils
import plot 
import cte_mov
import pid 

if __name__ == "__main__":

    try:
        rospy.init_node("controller", anonymous=True)
        #setting FSM_state
        #goes into ControlOp
        fsm.FSM_state().set_FSM_state(4)

        # je pense que c'est inutile ici ça
        #initialization applying a constant torque
        # pas assez stable, à améliorer
        # init = InitMove.init_mov()
        # init.run(True)
        # rospy.loginfo("init successful")

        #choose the controller

        # PID controller
        # pid.pid().run(2)
        
        #constant torque controller
        # start the movement. For now same traj 2 times
        cte_mov.cte_mov().run(2)

    except rospy.ROSInterruptException:
        utils.utils().stop()

        # TODO set a clock => ptetre pas nécessaire. a voir 
