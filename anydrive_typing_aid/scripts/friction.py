#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import pandas as pd
from statistics import mean

import utils

global JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE = 10


class Friction:
    def __init__(self):
        self.p_des, self.v_des, self.t_des, self.t_next, self.avg_v = 0, 0, 0, 0, 0
        self.u = utils.utils()
        self.t_meas_, self.v_meas_, self.p_meas_, self.t_next_ = [], [], [], []

        self.param = {
            "rate": 25,  # in hz
            "tau_0": 0.4,
            "tau_down": 0.0,
            "tau_max": 1.0,
            "v_des": 3,
            "d_tau_up_per_sec": 0.4,
            "d_tau_down_per_sec": 0.4,
            "tau_min": 0.0,
            "tau_max": 1.0,
        }
        self.u.save_param(self.param, "friction")

        self.d_tau_up = self.param["d_tau_up_per_sec"] / self.param["rate"]
        self.d_tau_down = self.param["d_tau_down_per_sec"] / self.param["rate"]

        self.move_up = False
        self.current_torque = self.param["tau_0"]

        self.sampling_time = 1.0 / self.param["rate"]
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

    def check_velocity(self, v_meas, v_des):
        if self.avg_vel():
            if abs(self.avg_v) >= v_des:
                return True
            else:
                return False
        else:
            return False

    def avg_vel(self):
        if len(self.v_meas_) < 5:
            return False
        else:
            self.avg_v = mean(self.v_meas_[-5:])
            return True

    def callback(self, msg):
        rospy.loginfo("Got triggered")
        self.move_up = True

    def stop(self):
        rospy.loginfo("Exit handler")
        # collecting the data
        self.u.concat_data(
            [],
            "",
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            self.t_next_,
            "desired torque",
            "_friction",
        )
        # # plotting the desired path
        # x = np.arange(0, len(self.t_meas_), 1)
        # self.u.plot(x, self.y , "desired_traj.png")
        # self.u.plot(x, self.t_meas_ , "torque.png")
        self.u.stop()

    def run(self):
        rospy.loginfo("starting movement")
        try:
            while not rospy.is_shutdown():
                t_meas, v_meas, p_meas = self.u.listener()
                self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(
                    t_meas, v_meas, p_meas, self.t_meas_, self.v_meas_, self.p_meas_
                )
                if self.move_up:
                    self.current_torque += self.d_tau_up
                    if self.current_torque > self.param["tau_max"]:
                        self.current_torque = self.param["tau_max"]
                    if self.check_velocity(v_meas, self.param["v_des"]):
                        self.move_up = False
                        print("Finished moving up")
                else:
                    self.current_torque -= self.d_tau_down
                    if self.current_torque < self.param["tau_0"]:
                        self.current_torque = self.param["tau_0"]

                print("Current torque: {}".format(self.current_torque))
                self.u.move(JOINT_TORQUE, self.p_des, self.v_des, self.current_torque)
                self.rate.sleep()

        except rospy.ROSInterruptException:
            self.u.stop()
