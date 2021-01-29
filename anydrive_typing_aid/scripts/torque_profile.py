#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import pandas as pd
import time

import fsmstate as fsm
import utils

global JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE = 10


class cte_mov:
    def __init__(self):
        self.p_des, self.v_des = 0, 0
        self.u = utils.utils()
        self.t_meas_, self.v_meas_, self.p_meas_, self.t_cmd_ = [], [], [], []

        self.param = {
            "t0": 0.0,
            "t_end": 1.0,
            "rate": 60,  # in hz
            "tau_0": 0.3,
            "tau_low": 0.0,
            "tau_end": 0.9,
            "transition": 0.4,
            "tau_min": -0.5,
            "tau_max": 2.0,
        }

        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("computing trajectory")
        self.other_traj = True
        self.x, self.y = self.compute_traj()
        self.u.plot(self.x, self.y, "desired_traj.png")
        self.total_steps = len(self.y)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0
        # choose the torque profile

    # transition time is the time needed to go from low torque to high torque
    # tau_0 is low torque value and tau_end is high torque value
    def compute_traj(self):
        x1, y1 = self.u.quadratic_fct(
            self.param["t0"],
            self.param["t0"] + self.param["transition"],
            self.param["tau_0"],
            self.param["tau_end"],
            self.sampling_time,
        )
        x2, y2 = self.u.const(
            self.param["tau_end"],
            self.param["t0"] + self.param["transition"],
            self.param["t_end"] - self.param["transition"],
            self.sampling_time,
        )
        if not self.other_traj:
            x3, y3 = self.u.quadratic_fct(
                self.param["t_end"] - self.param["transition"],
                self.param["t_end"],
                self.param["tau_end"],
                self.param["tau_0"],
                self.sampling_time,
            )
            # put everything together
            x, y = self.u.torque_profile(y1, y2, y3, x1, x2, x3)
        else:
            half_time = (
                self.param["transition"] / 2.0
                + self.param["t_end"]
                - self.param["transition"]
            )
            x3, y3 = self.u.quadratic_fct(
                self.param["t_end"] - self.param["transition"],
                half_time,
                self.param["tau_end"],
                self.param["tau_low"],
                self.sampling_time,
            )
            x4, y4 = self.u.quadratic_fct(
                half_time,
                self.param["t_end"],
                self.param["tau_low"],
                self.param["tau_0"],
                self.sampling_time,
            )
            x5, y5 = self.u.torque_profile(y1, y2, y3, x1, x2, x3)
            x, y = self.u.add_profile(x5, y5, x4, y4)
        return x, y

    def callback(self, msg):
        rospy.loginfo("Got triggered")
        if self.steps_left > 0:
            return
        self.steps_left = self.total_steps

    def stop(self):
        rospy.loginfo("Exit handler")
        self.u.save_param(self.param, "torque_profile", "torque_profile/")
        # collecting the data
        self.u.concat_data(
            self.t_cmd_,
            "commanded torque",
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            "_cte_mov",
            "torque_profile/",
        )
        # # plotting the desired path
        # x = np.arange(0, len(self.t_meas_), 1)
        # self.u.plot(x, self.y , "desired_traj.png")
        # self.u.plot(x, self.t_meas_ , "torque.png")

        self.u.stop_drive()

    def run(self):
        rospy.loginfo("starting movement")
        try:
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    # Moving up
                    t_cmd = self.y[self.total_steps - self.steps_left]
                    self.steps_left -= 1
                else:
                    t_cmd = self.param["tau_0"]
                self.u.move(JOINT_TORQUE, self.p_des, self.v_des, t_cmd)
                self.t_cmd_ = self.u.store_one(t_cmd, self.t_cmd_)
                t_meas, v_meas, p_meas = self.u.listener()
                self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(
                    t_meas, v_meas, p_meas, self.t_meas_, self.v_meas_, self.p_meas_
                )
                # if self.u.lim_check(self.param):
                #     raise rospy.ROSInterruptException
                self.rate.sleep()

        except rospy.ROSInterruptException:
            self.u.stop_drive()
