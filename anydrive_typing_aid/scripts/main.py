#!/usr/bin/env python
# coding=utf-8

import rospy
import fsmstate as fsm
import InitMove
import utils
import plot
import cte_mov

if __name__ == "__main__":

    try:
        rospy.init_node("controller", anonymous=True)
        #setting FSM_state
        #goes into ControlOp
        fsm.FSM_state().set_FSM_state(4)

        #initialization applying a constant torque
        # pas assez stable, à améliorer
        # init = InitMove.init_mov()
        # init.run(True)
        rospy.loginfo("init successful")

        #choose the controller
        # PID controller
        # utils().pid
        
        #constant torque controller
        rate = rospy.Rate(200) # 200hz
        c = cte_mov.cte_mov()
        # computing the torque trajectory
        # need to set the values inside the brackets
        rospy.loginfo("computing trajectory")
        t0 = 0
        t_end = 10
        tau_0 = 0.3
        tau_end = 0.7
        transition = 3
        x,y = c.compute_traj(t0,t_end, tau_0, tau_end, transition)
        # start the movement. For now same traj indeinitely
        rospy.loginfo("starting movement")
        while not rospy.is_shutdown():
            c.move(y, rate, 2)
        # stop the drive at the end of the loop
        utils.utils().stop()

        # # draw the plots
        # p = plot()
        # i = input("number of plots wanted")
        # while i > 0:
        #     x, y, ylabel = p.arg()
        #     p.plot(x, y, ylabel)
        #     i = i-1

    except rospy.ROSInterruptException:
        utils.utils().stop()

        # TODO do the plots of torque velocity and position
        # TODO set a clock 
