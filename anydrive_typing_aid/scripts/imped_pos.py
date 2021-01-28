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
            "x_end": 0.14819693565368652,
            "x_0_lim": -3.5230984687805176,
            "x_end_lim": 3.2660608291625977,
            "x_0": -2.897282361984253,
            "tau_0": 0.3,
            "transition": 1,
            "K": 0.7,
            "tau_min": 0,
            "tau_max": 3,
        }

        # self.param = self.u.set_pos(self.param)
        self.u.save_param(self.param, "imped_pos")

        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0

    # transition time is the time needed to go from low torque to high torque
    # x_0 is low torque value and x_end is high torque value
    def compute_traj(self, p_meas):
        # way up :
        x1, y1 = self.u.quadratic_fct(
            self.param["t0"],
            self.param["t0"] + self.param["transition"],
            p_meas,
            self.param["x_end"],
            self.sampling_time,
        )
        x2, y2 = self.u.const(
            self.param["x_end"],
            self.param["t0"] + self.param["transition"],
            self.param["t_end"] - self.param["transition"],
            self.sampling_time,
        )
        x3, y3 = self.u.quadratic_fct(
            self.param["t_end"] - self.param["transition"],
            self.param["t_end"],
            self.param["x_end"],
            self.param["x_0"],
            self.sampling_time,
        )
        # put everything together
        return self.u.torque_profile(y1, y2, y3, x1, x2, x3)

    def callback(self, msg):
        if self.steps_left > 0:
            return
        rospy.loginfo("Got triggered")
        _, _, p_meas = self.u.listener()
        self.x, self.y = self.compute_traj(p_meas)
        self.u.plot(self.x, self.y, "desired_traj.png")
        self.total_steps = len(self.y)
        self.steps_left = self.total_steps

    def stop(self):
        rospy.loginfo("Exit handler")
        # collecting the data
        self.u.concat_data(
            self.y,
            "desired position",
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            self.t_cmd_,
            "commanded torque",
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
