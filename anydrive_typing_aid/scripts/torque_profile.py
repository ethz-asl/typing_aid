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
        self.t_meas_, self.v_meas_, self.p_meas_, self.i_meas_, self.t_cmd_ = (
            [],
            [],
            [],
            [],
            [],
        )

        self.param = {
            "transition_up": 0.5,
            "duration_constant_up": 0.0,
            "transition_down": 0.5,
            "rate": 80,  # in hz
            "tau_0": 0.5,
            "tau_low": 0.0,
            "tau_end": 1.5,
            "tau_min": -1.0,
            "tau_max": 3.0,
            "x_0_lim": -3.8042964935302734,
            "x_end_lim": 4.61220121383667,
        }
        self.u.save_param(self.param, "torque_profile", "torque_profile/")
        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("computing trajectory")
        self.other_traj = True
        self.x, self.y = self.u.compute_traj(
            self.param,
            "tau_0",
            -99,
            "tau_end",
            "tau_low",
            self.other_traj,
            self.sampling_time,
        )
        self.u.plot(self.x, self.y, "desired_traj.png")
        self.total_steps = len(self.y)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0
        # choose the torque profile

    def callback(self, msg):
        rospy.loginfo("Got triggered")
        # if self.steps_left > 0:
        #     return
        self.steps_left = self.total_steps

    def stop(self):
        rospy.loginfo("Exit handler")

        # collecting the data
        self.u.concat_data(
            self.t_cmd_,
            "commanded torque",
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            self.i_meas_,
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
                t_meas, v_meas, p_meas, i_meas = self.u.listener()
                self.t_meas_, self.v_meas_, self.p_meas_, self.i_meas_ = self.u.store(
                    t_meas,
                    v_meas,
                    p_meas,
                    i_meas,
                    self.t_meas_,
                    self.v_meas_,
                    self.p_meas_,
                    self.i_meas_,
                )
                if self.u.lim_check(self.param, self.t_meas_, p_meas):
                    raise rospy.ROSInterruptException
                self.rate.sleep()

        except rospy.ROSInterruptException:
            self.stop()
