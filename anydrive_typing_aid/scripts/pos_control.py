#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header, Empty
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
        self.p_des,self.v_des, self.t_des = 0,0,0 
        self.u = utils.utils()
        self.t_meas_, self.v_meas_, self.p_meas_ = [],[],[]

        self.param = {
            "t0" : 0.0,
            "t_end" : 1.0,
            "rate" :20,
            "x_0" : None,
            "x_end" : None,
            "x_end_lim" : None, 
            "x_0_lim" : None,
            "transition" : 0.2
        }
        self.param = self.u.set_pos(self.param)
        
        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("computing trajectory")
        self.x,self.y = self.compute_traj()
        self.total_steps = len(self.y)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0

# transition time is the time needed to go from low pos to high pos
# x_0 is low torque value and x_end is high torque value
    def compute_traj(self):
        params = self.set_param()
        # way up : 
        x1,y1 = self.u.quadratic_fct(params["t0"], params["t0"]+params["transition"], params["x_0"], params["x_end"],params["num"])
        x2,y2 = self.u.const(params["x_end"], params["t0"]+params["transition"] , params["t_end"]-params["transition"])
        x3,y3 = self.u.quadratic_fct(params["t_end"]-params["transition"],params["t_end"], params["x_end"], params["x_0"],params["num"])
        # put everything together
        return self.u.torque_profile(y1,y2,y3,x1,x2,x3)

    def callback(self, msg):
        rospy.loginfo("Got triggered")
        if self.steps_left > 0:
            return
        self.steps_left = self.total_steps

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
                if self.u.lim_check(position):
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

    def run(self):
        rospy.loginfo("starting movement")
        try:
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    # Moving up
                    self.p_des = self.y[self.total_steps - self.steps_left]
                    self.u.move(JOINT_POSITION,self.p_des, self.v_des, self.t_des)
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
            data_pd.to_csv(name +"_pos_control.csv")

            # # plotting the desired path
            # x = np.arange(0, len(self.t_meas_), 1)
            # self.u.plot(x, self.y , "desired_traj.png")
            # self.u.plot(x, self.t_meas_ , "torque.png")

        except rospy.ROSInterruptException:
            self.u.stop()