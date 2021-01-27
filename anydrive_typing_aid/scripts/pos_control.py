#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import pandas as pd
import numpy as np

import fsmstate as fsm
import utils
from ctrl_common import BaseController

global JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE, JOINT_POSITION_VELOCITY
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE = 10
JOINT_POSITION_VELOCITY = 11


class pos_mov(BaseController):
    def __init__(self):
        super(pos_mov, self).__init__()

        self.p_des, self.v_des, self.t_des, self.v_last = 0, 0, 0, 0
        self.t_meas_, self.v_meas_, self.p_meas_, self.dy = [], [], [], []

        self.param = {
            "t0": 0.0,
            "t_end": 7.0,
            "x_end": 2.649951934814453,
            "x_0_lim": -2.3559787273406982,
            "x_end_lim": 5.186628818511963,
            "x_0": 0.40113598108291626,
            "transition": 2.0,
            "rate": self.rate_hz,
            "tau_0": 0.5,
            "p": 2,
            "i": 0.078,
            "d": 0.163,
            "tau_min": -1,
            "tau_max": 2,
        }
        # self.param = self.u.set_pos(self.param)
        # print(self.param)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.steps_left = 0

    def callback(self, msg):
        if self.steps_left > 0:
            return
        rospy.loginfo("Got triggered")
        _, _, p_meas = self.u.listener()
        self.x, self.y = self.compute_traj(p_meas)
        self.dy = np.diff(self.y) * self.rate_hz
        self.u.plot(self.x, self.y, "desired_traj.png")
        self.u.plot(self.x[:-1], self.dy, "desired_vel.png")
        self.total_steps = len(self.y)
        self.steps_left = self.total_steps

    def stop(self):
        print("Exit handler")
        # saving the data
        self.u.concat_data(
            self.y,
            "desired position",
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            [],
            "",
            "_pos_control",
        )
        # # plotting the desired path
        # x = np.arange(0, len(self.t_meas_), 1)
        # self.u.plot(x, self.y , "desired_traj.png")
        # self.u.plot(x, self.t_meas_ , "torque.png")
        self.u.stop()

    def run(self):
        rospy.loginfo("starting movement")
        try:

            # change pid gains
            self.u.pid(JOINT_POSITION, self.param)
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    # Moving up
                    self.p_des = self.y[self.total_steps - self.steps_left]
                    self.v_des = self.dy[self.total_steps - self.steps_left]
                    self.u.move(
                        JOINT_POSITION_VELOCITY, self.p_des, self.v_des, self.t_des
                    )
                    self.steps_left -= 1
                else:
                    self.u.move(
                        JOINT_TORQUE, self.p_des, self.v_des, self.param["tau_0"]
                    )
                t_meas, v_meas, p_meas = self.u.listener()
                self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(
                    t_meas, v_meas, p_meas, self.t_meas_, self.v_meas_, self.p_meas_
                )
                if self.u.lim_check(self.param):
                    raise rospy.ROSInterruptException
                self.rate.sleep()

        except rospy.ROSInterruptException:
            self.u.stop()