#!/usr/bin/env python
# coding=utf-8

# idea is to create a pid with position as input and torque as output for the going down movement.

import math
import rospy
import numpy as np
import time
import pandas as pd
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs

import utils

global JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE = 10


class pid:
    def __init__(self):
        self._i_error, self._d_error = 0, 0
        self._p_gain, self._i_gain, self._d_gain = None, None, None
        self._last_time, self._p_error_last = None, None
        self.u = utils.utils()

        (
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            self.i_meas_,
            self._p_error_,
            self.t_cmd_,
        ) = ([], [], [], [], [], [])
        self.p_, self.i_, self.d_ = [], [], []
        self.p_des, self.v_des = 0, 0

        self.param = {
            "transition_up": 0.5,
            "duration_constant_up": 0.0,
            "transition_down": 0.5,
            "rate": 25,  # in hz
            "x_end": 1.3717384338378906,
            "x_0_lim": -3.8042964935302734,
            "x_end_lim": 4.61220121383667,
            "x_0": -2.6006247997283936,
            "tol": 0.7,
            "increment": 0.3,
            "tau_0": 0.5,
            "tau_min": -1.0,
            "tau_max": 3.0,
            "p_gain": 1.1,
            "i_gain": 0.5,
            "d_gain": 0.5,
        }

        # self.param = self.u.set_PID(self.param)
        self.param = self.u.set_pos(self.param)
        print(self.param)

        self.u.save_param(self.param, "pid", "pid/")
        _, _, p_meas, _ = self.u.listener()
        self.p_error = p_meas - self.param["x_end"]
        self.set_gains()

        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0
        self.other_traj = True

    def set_gains(self):
        self._p_gain = self.param["p_gain"]
        self._i_gain = self.param["i_gain"]
        self._d_gain = self.param["d_gain"]

    def update(self, p_error, wind_val=0):
        if self._last_time is None:
            self._last_time = 0
        dt = self.sampling_time - self._last_time

        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0

        self._p_error = self.p_error
        p = self._p_gain * self._p_error

        self._i_error += dt * self._p_error
        i = self._i_gain * self._i_error

        i = self.antiWindup(i, wind_val)

        if self._p_error_last is None:
            self._p_error_last = 0
        else:
            self._p_error_last = self._p_error
        self._d_error = (self._p_error - self._p_error_last) / dt

        d = self._d_gain * self._d_error

        self.p_, self.i_, self.d_ = self.u.store_pid(p, i, d, self.p_, self.i_, self.d_)

        self.cmd = p + i + d
        return self.cmd

    # t_meas is the measured curent torque, t_cmd is the desired torque sent to the drive, tol is the maximum delta we allow
    # defined in self.param
    def filt(self, t_meas, t_cmd):
        # if true, values are out of bounds
        if self.u.lim_check(self.param, self.t_meas_, self.p_meas_[-1]):
            rospy.loginfo("value out of bounds")
            t_cmd = self.u.get_lim_val(t_meas, self.param)
        if abs(t_meas - t_cmd) >= self.param["tol"]:
            cte = self.u.check_sign(t_meas, t_cmd)
            rospy.loginfo("step to big, adapting")
            t_cmd = t_meas + cte * self.param["tol"]
        return t_cmd

    def antiWindup(self, i, wind_val):
        if i >= wind_val:
            i = wind_val
        elif i <= -wind_val:
            i = -wind_val
        return i

    def callback(self, msg):
        rospy.loginfo("Got triggered")
        if self.steps_left > 0:
            return
        rospy.loginfo("computing trajectory")
        _, _, p_meas, _ = self.u.listener()
        self.x, self.y = self.u.compute_traj(
            self.param,
            "x_0",
            p_meas,
            "x_end",
            "x_0_lim",
            self.other_traj,
            self.sampling_time,
        )
        self.total_steps = len(self.y)
        self.steps_left = self.total_steps

    def stop(self):
        rospy.loginfo("Exit handler")

        # concatenating the data
        print("length1: {}".format(len(self.t_cmd_)))
        print("length2: {}".format(len(self.t_meas_)))
        data_concat = np.array(
            (
                self.t_cmd_[: len(self.t_meas_)],
                self.t_meas_,
                self.v_meas_,
                self.p_meas_,
                self.i_meas_,
            )
        ).T
        data_pd = pd.DataFrame(
            data=data_concat,
            columns=["commanded torque", "torque", "velovity", "position", "current"],
        )
        print("length1: {}".format(len(self.p_)))
        print("length2: {}".format(len(self._p_error_)))

        pid_concat = np.array(
            (self.p_, self.i_, self.d_, self._p_error_[: len(self.d_)])
        ).T
        pid_pd = pd.DataFrame(
            data=pid_concat,
            columns=["p gain", "i gain", "d gain", "position error"],
        )

        all_in = pd.concat([data_pd, pid_pd], axis=1)
        name = self.u.get_time()
        path = "/home/asl-admin/Desktop/pid/"
        all_in.to_csv(path + name + "_pid.csv")
        # x = np.arange(0, len(self.t_meas_), 1)
        # self.u.plot(x, self.y , "desired_traj.png")
        # self.u.plot(x, self.t_meas_ , "torque.png")
        # self.u.plot(x, self.p_error_ , "position_error.png")
        self.u.stop_drive()

    def run(self):
        rospy.loginfo("starting movement")
        try:
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    # Moving up
                    t_meas, v_meas, p_meas, _ = self.u.listener()
                    self.p_error = p_meas - self.y[self.total_steps - self.steps_left]
                    self._p_error_ = self.u.store_one(self.p_error, self._p_error_)
                    t_cmd = self.update(self.p_error)
                    t_cmd = self.filt(t_meas, t_cmd)
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