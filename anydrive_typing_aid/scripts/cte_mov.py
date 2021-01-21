#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import pandas as pd
import time
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
                "t_max":2
            }

class cte_mov:
    def __init__(self):
        self.p_des,self.v_des, self.t_des = 0,0,0
        self.u = utils.utils()
        self.t_meas_, self.v_meas_, self.p_meas_ = [],[],[]

        self.param = {
            "t0" : 0.0,
            "t_end" : 1.0,
            "rate" : 25, # in hz
            "tau_0" : 0.5,
            "tau_end" : 0.9,
            "transition" : 0.2
        }

        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("computing trajectory")
        self.x,self.y = self.compute_traj()
        self.total_steps = len(self.y)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0

# transition time is the time needed to go from low torque to high torque
# tau_0 is low torque value and tau_end is high torque value
    def compute_traj(self):
        # way up : 
        x1,y1 = self.u.quadratic_fct(self.param["t0"], self.param["t0"]+self.param["transition"], self.param["tau_0"], self.param["tau_end"], self.sampling_time)
        x2,y2 = self.u.const(self.param["tau_end"], self.param["t0"]+self.param["transition"] , self.param["t_end"]-self.param["transition"],self.sampling_time)
        x3,y3 = self.u.quadratic_fct(self.param["t_end"]-self.param["transition"],self.param["t_end"], self.param["tau_end"], self.param["tau_0"],self.sampling_time)
        # put everything together
        return self.u.torque_profile(y1,y2,y3,x1,x2,x3)

    def callback(self, msg):
        rospy.loginfo("Got triggered")
        if self.steps_left > 0:
            return
        self.steps_left = self.total_steps

    # # n is the number of times the path is taken
    # def move(self, n):
    #     rate_hz = self.param["rate"]
    #     rate = rospy.Rate(rate_hz)
    #     rospy.loginfo("computing trajectory")
    #     x,y = self.compute_traj()
        
    #     while n>=1: 
    #         l = 0 
    #         while l <= (len(y)-1):
    #             # p_des is unsused here, just defined to make it worked
    #             p_des =0
    #             t_next = y[l]
    #             self.u.move(JOINT_TORQUE,p_des, self.v_des, t_next)
    #             t_meas, v_meas, p_meas = self.u.listener()
    #             # rospy.loginfo("applied torque: {}".format(t_next))
    #             if self.u.lim_check(position):
    #                 raise rospy.ROSInterruptException
    #             self.t_meas_,self.v_meas_,self.p_meas_ = self.u.store(t_meas, v_meas, p_meas,self.t_meas_, self.v_meas_, self.p_meas_)
    #             l+=1
    #             rate.sleep()
    #         n = n-1
    #     # plotting the desired path
    #     x = np.arange(0, len(self.t_meas_), 1)
    #     self.u.plot(x, y , "desired_traj.png")
    #     self.u.plot(x, self.t_meas_ , "torque.png")

    #     #concatenating the data
    #     data_concat = np.array((y,self.t_meas_,self.v_meas_,self.p_meas_)).T
    #     data_pd = pd.DataFrame(data=data_concat, columns=("commanded torque","torque","velovity","position"))
    #     data_pd.to_csv("test_out.csv")

    def run(self):
        rospy.loginfo("starting movement")
        try:
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    # Moving up
                    t_next = self.y[self.total_steps - self.steps_left]
                    self.u.move(JOINT_TORQUE,self.p_des, self.v_des, t_next)
                    self.steps_left -= 1
                else:
                    self.u.move(JOINT_TORQUE,self.p_des, self.v_des, self.param["tau_0"])
                t_meas, v_meas, p_meas = self.u.listener()
                self.t_meas_,self.v_meas_,self.p_meas_ = self.u.store(t_meas, v_meas, p_meas,self.t_meas_, self.v_meas_, self.p_meas_)
                # TODO changer ce position. Peut être faire une classe avec les params de chaque cas
                if self.u.lim_check(position):
                    raise rospy.ROSInterruptException
                self.rate.sleep()

            #concatenating the data
            data_concat = np.array((self.y,self.t_meas_,self.v_meas_,self.p_meas_)).T
            data_pd = pd.DataFrame(data=data_concat, columns=("commanded torque","torque","velovity","position"))
            name = self.u.get_time()
            data_pd.to_csv(name +"_cte_mov.csv")

            # # plotting the desired path
            # x = np.arange(0, len(self.t_meas_), 1)
            # self.u.plot(x, self.y , "desired_traj.png")
            # self.u.plot(x, self.t_meas_ , "torque.png")

        except rospy.ROSInterruptException:
            self.u.stop()

# TODO check the limitations 