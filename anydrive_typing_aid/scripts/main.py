#!/usr/bin/env python
# coding=utf-8

import rospy

import fsmstate as fsm
import InitMove
import utils
import plot 
import cte_mov
import pid 
import pos_control as p_cont
import saver

if __name__ == "__main__":

    try:
        rospy.init_node("controller", anonymous=True)
        #setting FSM_state
        #goes into ControlOp
        fsm.FSM_state().set_FSM_state(4)
        # gonna run the chosen control method and output the saved values in a csv file
        saver.save().method_selection()

        # je pense que c'est inutile ici ça
        #initialization applying a constant torque
        # pas assez stable, à améliorer, ça sert à rien, peut etre supprime
        # init = InitMove.init_mov()
        # init.run(True)
        # rospy.loginfo("init successful")

        #choose the controller

        # PID controller
        # pid.pid().run(2)
        
        #constant torque controller
        # start the movement. For now same traj 2 times
        # cte_mov.cte_mov().run(1)

    except rospy.ROSInterruptException:
        utils.utils().stop()

        # TODO set a clock => ptetre pas nécessaire. a voir 
