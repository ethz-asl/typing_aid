#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
from scipy import signal
import fsmstate as fsm
import pid
import utils

global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10

position = {
                "up_position": 4,
                "down_position": 0,
                "up_limit": 5,
                "down_limit": -2.5,
                "v_max": 5,
                "t_min":0,
                "t_max":1
            }

class cte_mov:
    def __init__(self):
        self.v_des, self.t_des = 0,0
        self.u = utils.utils()

# transition time is the time needed to go from low torque to high torque
# tau_0 is low torque value and tau_end is high torque value
    def compute_traj(self,t0,t_end, tau_0, tau_end, duration):
        # way up : 
        x1,y1 = self.u.quadratic_fct(t0, t0+duration, tau_0, tau_end)
        x2,y2 = self.u.const(tau_end, t0+duration , t_end-duration)
        x3,y3 = self.u.quadratic_fct(t_end-duration,t_end, tau_end, tau_0)
        # put everything together
        return self.u.torque_profile(y1,y2,y3,x1,x2,x3)

    # n is the number of times the path is taken
    def move(self, y, rate, n):
        l = 0 
        while n>=0: 
            while l <= (len(y)-1):
                # p_des is unsused here, just defined to make it worked
                p_des =0
                t_next = y[l]
                self.u.move(JOINT_TORQUE,p_des, self.v_des, t_next)
                t_meas, v_meas, p_meas = self.u.listener()
                rospy.loginfo("applied torque: {}".format(t_next))
                if self.u.lim_check(l,position):
                    raise rospy.ROSInterruptException
                l+=1
                rate.sleep()
                l+=1
            n = n-1
            l = 0
        
# duration variable*2 = duration to type on the tablet. If needed, can add a cte line for down torque in beteen 2 cycles
# TODO check the limitations 
# TODO adapt the time to seconds !!!