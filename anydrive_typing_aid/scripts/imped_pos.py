#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import pandas as pd

import utils


global JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE = 10


class impedance:
    def __init__(self):
        self.p_des, self.v_des = 0, 0
        self.u = utils.utils()
        self.t_meas_, self.v_meas_, self.p_meas_, self.t_cmd_ = [], [], [], []

        self.param = {
            "t0": 0.0,
            "t_end": 2.0,
            "rate": 25,  # in hz
            "x_end": 9.015493392944336,
            "x_0_lim": 2.0243513584136963,
            "x_end_lim": 12.032234191894531,
            "x_0": 4.290712356567383,
            "tau_0": 0.3,
            "transition": 1,
            "K": 0.7,
            "tau_min": 0,
            "tau_max": 3,
        }

        # self.param = self.u.set_pos(self.param)

        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)
        self.other_traj = False

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0

    def callback(self, msg):
        if self.steps_left > 0:
            return
        rospy.loginfo("Got triggered")
        _, _, p_meas = self.u.listener()
        self.x, self.y = self.u.compute_traj(
            self.param, "x_0", p_meas, "x_end", _, self.other_traj, self.sampling_time
        )
        self.u.plot(self.x, self.y, "desired_traj.png")
        self.total_steps = len(self.y)
        self.steps_left = self.total_steps

    def stop(self):
        rospy.loginfo("Exit handler")
        self.u.save_param(self.param, "imped_pos", "imped_pos/")
        # collecting the data
        self.u.concat_data(
            self.t_cmd_,
            "commanded torque",
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            "_imped",
            "imped_pos/",
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
                    t_meas, v_meas, p_meas = self.u.listener()
                    t_cmd = self.param["K"] * (
                        self.y[self.total_steps - self.steps_left] - p_meas
                    )
                    if t_cmd < self.param["tau_0"]:
                        t_cmd = self.param["tau_0"]
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
