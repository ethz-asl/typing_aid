#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import fsmstate as fsm
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

class pos_mov:
    def __init__(self):
        self.v_des, self.t_des = 0,0
        self.u = utils.utils()
        self.t_meas_, self.v_meas_, self.p_meas_, self.data = [],[],[],[]
    
    def set_param(self):
        params = {
            "t0" : 0,
            "t_end" : 10,
            "rate" : rospy.Rate(20), # in hz
            # TODO measure these values and put them here
            "p_0" : 4,
            "p_end" : 10,
            "transition" : 3
        }
        return params

# transition time is the time needed to go from low pos to high pos
# p_0 is low torque value and p_end is high torque value
    def compute_traj(self):
        params = self.set_param()
        # way up : 
        x1,y1 = self.u.quadratic_fct(params["t0"], params["t0"]+params["transition"], params["p_0"], params["p_end"],position["rate"])
        x2,y2 = self.u.const(params["p_end"], params["t0"]+params["transition"] , params["t_end"]-params["transition"])
        x3,y3 = self.u.quadratic_fct(params["t_end"]-params["transition"],params["t_end"], params["p_end"], params["p_0"],position["rate"])
        # put everything together
        return self.u.torque_profile(y1,y2,y3,x1,x2,x3)

    # n is the number of times the path is taken
    def move(self, n):
        rospy.loginfo("getting parameters")
        params = self.set_param()
        rate = params["rate"]
        rospy.loginfo("computing trajectory")
        x,y = self.compute_traj()
        l = 0 
        while n>=1: 
            while l <= (len(y)-1):
                # t_next is unsused here, just defined to make it worked
                p_des = y[l]
                t_next = 0
                self.u.move(JOINT_POSITION,p_des, self.v_des, t_next)
                t_meas, v_meas, p_meas = self.u.listener()
                rospy.loginfo("desired position: {}".format(t_next))
                if self.u.lim_check(l,position):
                    raise rospy.ROSInterruptException
                self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(t_meas, v_meas, p_meas,self.t_meas_, self.v_meas_, self.p_meas_)
                l+=1
                rate.sleep()
            n = n-1
            l = 0
        # plotting the desired path
        x = np.arange(0, len(self.t_meas_), 1)
        self.u.plot(x, y , "desired_traj.png")
        self.u.plot(x, self.p_meas_ , "position.png")

        #concatenating the data
        self.data = s.save().add_data_col([self.t_meas_,self.v_meas_,self.p_meas_], ax = 0)

    def run(self, n):
        rospy.loginfo("starting movement")
        self.move(n)
        self.u.stop()
# TODO check the limitations 