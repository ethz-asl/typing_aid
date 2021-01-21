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
position = {
    "up_position": 4,
    "down_position": 0,
    "up_limit": 5,
    "down_limit": -2.5,
    "v_max": 5,
    "t_min": 0,
    "t_max": 1,
}


class pid:
    def __init__(self):
        self._p_error, self._i_error, self._d_error = 0, 0, 0
        self._p_gain, self._i_gain, self._d_gain = None, None, None
        self._last_time, self._p_error_last = None, None
        self.u = utils.utils()
        p_gain, i_gain, d_gain, self.p_error = self.u.set_PID()
        self.set_gains(p_gain, i_gain, d_gain)
        self.t_meas_, self.v_meas_, self.p_meas_, self._p_error_ = [], [], [], []
        self.p_, self.i_, self.d_ = [], [], []
        self.p_des, self.v_des = 0, 0

        self.param = {
            "t0": 0,
            "t_end": 10,
            "rate": 25,  # in hz
            "num": 200,
            "v_0": 0.3,
            "v_end": 0.7,
            "transition": 3,
            "v_min": -0.3,
            "v_max": 0.5,
            "t_min": 0.25,
            "t_max": 0.75,
            "tol": 0.01,
            "tau_0": 0.5,
        }
        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)
        rospy.loginfo("computing trajectory")
        self.x, self.y = self.compute_traj()
        self.total_steps = len(self.y)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0

    def set_gains(self, p_gain, i_gain, d_gain):
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain

    # transition time is the time needed to go from low torque to high torque
    # tau_0 is low torque value and tau_end is high torque value
    def compute_traj(self):
        # way up :
        x1, y1 = self.u.quadratic_fct(
            self.param["t0"],
            self.param["t0"] + self.param["transition"],
            self.param["v_0"],
            self.param["v_end"],
            self.sampling_time,
        )
        x2, y2 = self.u.const(
            self.param["v_end"],
            self.param["t0"] + self.param["transition"],
            self.param["t_end"] - self.param["transition"],
            self.sampling_time,
        )
        x3, y3 = self.u.quadratic_fct(
            self.param["t_end"] - self.param["transition"],
            self.param["t_end"],
            self.param["v_end"],
            self.param["v_0"],
            self.sampling_time,
        )
        # put everything together
        return self.u.torque_profile(y1, y2, y3, x1, x2, x3)

    def update(self, p_error, wind_val=0):
        t = time.time()
        if self._last_time is None:
            self._last_time = t
        dt = t - self._last_time

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

        self.p_, self.i_, self.d_ = self.u.store(p, i, d, self.p_, self.i_, self.d_)

        self.cmd = p + i + d
        return self.cmd

    # t_meas is the measured curent torque, t_next is the desired torque sent to the drive, tol is the maximum delta we allow
    # defined in self.param
    def filt(self, t_meas, t_next):
        # if true, values are out of bounds
        if self.u.lim_check(self.param):
            rospy.loginfo("value out of bounds")
            t_next = self.u.get_lim_val(t_meas, self.param)
        if abs(t_meas - t_next) >= self.param["tol"]:
            cte = self.u.check_sign(t_meas, t_next)
            rospy.loginfo("step to big, adapting")
            t_next = t_meas + cte * self.param["tol"]
        return t_next

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
        self.steps_left = self.total_steps

    # # n is the number of time the trajectory path (of velocity) is taken
    #     def move(self,n):
    #         # set the PID gains and pid_error to 0 and does one update
    #         self.u.set_PID()
    #         rospy.loginfo("getting parameters")
    #         self.param = self.set_param()
    #         rate = self.param["rate"]
    #         rospy.loginfo("computing trajectory")
    #         x,p_des = self.compute_traj()
    #         # define some velocity input profile
    #         l = 0
    #         while n>=1:
    #             while l <= (len(y)-1):
    #                 # x, v_des = put the function here for the position
    #                 t_meas, v_meas, p_meas = self.u.listener()
    #                 self.p_error = p_meas - p_des[l]
    #                 t_next = self.update(self.p_error)
    #                 t_next = self.filt(t_meas,t_next)
    #                 self.u.move(JOINT_TORQUE,p_des, self.v_des, t_next)
    #                 # rospy.loginfo("applied torque: {}".format(t_next))
    #                 if self.u.lim_check(self.param):
    #                     raise rospy.ROSInterruptException
    #                 # store the values
    #                 # l à changer parce que de 0 à 10 pour l'instant
    #                 self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(t_meas, v_meas, self.p_error,self.t_meas_, self.v_meas_, self.p_meas_)
    #                 l+=1
    #                 rate.sleep()
    #             n = n-1
    #             l = 0

    #     # plotting the desired path
    #         x = np.arange(0, len(self.t_meas_), 1)
    #         self.u.plot(x, p_des , "desired_traj.png")
    #         self.u.plot(x, self.t_meas_ , "torque.png")
    #         self.u.plot(x, self.p_error_ , "velocity_error.png")

    #     # saving the data
    #         self.data = s.save().add_data_col([self.t_meas_,self.v_meas_,self.p_meas_], ax = 0)
    #         # saves pid gains in another row (=below)
    #         self.data = s.save().add_data_col([self.p_,self.i_,self.d_], ax = 0)

    def run(self):
        rospy.loginfo("starting movement")
        try:
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    # Moving up
                    t_meas, v_meas, p_meas = self.u.listener()
                    self.p_error = p_meas - self.y[self.total_steps - self.steps_left]
                    t_next = self.update(self.p_error)
                    t_next = self.filt(t_meas, t_next)
                    self.u.move(JOINT_TORQUE, self.p_des, self.v_des, t_next)
                    self.steps_left -= 1
                else:
                    self.u.move(
                        JOINT_TORQUE, self.p_des, self.v_des, self.param["tau_0"]
                    )
                t_meas, v_meas, p_meas = self.u.listener()
                self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(
                    t_meas, v_meas, p_meas, self.t_meas_, self.v_meas_, self.p_meas_
                )
                self._p_error_ = self.u.store_one(self.p_error, self._p_error_)
                # TODO changer ce position. Peut être faire une classe avec les params de chaque cas
                if self.u.lim_check(position):
                    raise rospy.ROSInterruptException
                self.rate.sleep()

            # concatenating the data
            data_concat = np.array((self.y, self.t_meas_, self.v_meas_, self.p_meas_)).T
            data_pd = pd.DataFrame(
                data=data_concat,
                columns=("commanded torque", "torque", "velovity", "position"),
            )

            pid_concat = np.array((self.p_, self.i_, self.d_, self._p_error_)).T
            pid_pd = pd.DataFrame(
                data=pid_concat,
                columns=("p gain", "i gain", "d gain", "position error"),
            )

            all_in = pd.concat([data_pd, pid_pd], axis=1)
            name = self.u.get_time()
            all_in.to_csv(name + "_pid.csv")

            # x = np.arange(0, len(self.t_meas_), 1)
            # self.u.plot(x, self.y , "desired_traj.png")
            # self.u.plot(x, self.t_meas_ , "torque.png")
            # self.u.plot(x, self.p_error_ , "position_error.png")

        except rospy.ROSInterruptException:
            self.u.stop()