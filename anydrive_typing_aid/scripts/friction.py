#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import pandas as pd
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

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


class Friction:
    def __init__(self):
        self.p_des, self.v_des, self.t_des, self.t_next = 0, 0, 0, 0
        self.u = utils.utils()
        self.t_meas_, self.v_meas_, self.p_meas_, self.t_next_ = [], [], [], []

        self.param = {
            "t0": 0.0,
            "t_end": 1.0,
            "rate": 25,  # in hz
            "tau_0": 0.4,
            "tau_max": 1.0,
            "transition": 0.2,
            "v_des": 1.4,
            "d_tau_up_per_sec": 0.2,
            "d_tau_down_per_sec": 0.2,
        }

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
        if abs(v_meas) >= v_des:
            return True
        else:
            return False

    def callback(self, msg):
        rospy.loginfo("Got triggered")
        self.move_up = True

    def stop(self):
        rospy.loginfo("Exit handler")
        self.u.stop()

    def run(self):
        rospy.loginfo("starting movement")
        try:
            while not rospy.is_shutdown():
                _, v_meas, _ = self.u.listener()
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

                # if self.steps_left > 0:
                #     # Moving up
                #     t_meas, v_meas, p_meas = self.u.listener()
                #     while not self.check_velocity(v_meas, self.param["v_des"]):
                #         self.t_next = t_meas + self.param["d_tau_up"]
                #         self.t_next_ = self.u.store_one(self.t_next, self.t_next_)
                #         self.u.move(JOINT_TORQUE, self.p_des, self.v_des, self.t_next)
                #     self.t_next = t_meas - self.param["d_tau_down"]
                #     self.t_next_ = self.u.store_one(self.t_next, self.t_next_)
                #     self.u.move(JOINT_TORQUE, self.p_des, self.v_des, self.t_next)
                #     self.steps_left -= 1
                # else:
                #     self.u.move(
                #         JOINT_TORQUE, self.p_des, self.v_des, self.param["tau_0"]
                #     )
                # t_meas, v_meas, p_meas = self.u.listener()
                # self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(
                #     t_meas, v_meas, p_meas, self.t_meas_, self.v_meas_, self.p_meas_
                # )
                # TODO changer ce position. Peut être faire une classe avec les params de chaque cas
                # if self.u.lim_check(position):
                #     raise rospy.ROSInterruptException
                self.rate.sleep()

            # concatenating the data
            # data_concat = np.array((self.y, self.t_meas_, self.v_meas_, self.p_meas_)).T
            # data_pd = pd.DataFrame(
            #     data=data_concat,
            #     columns=("commanded torque", "torque", "velovity", "position"),
            # )

            # pid_concat = np.array((self.t_next_)).T
            # pid_pd = pd.DataFrame(data=pid_concat, columns=("desired torque"))

            # all_in = pd.concat([data_pd, pid_pd], axis=1)
            # name = self.u.get_time()
            # all_in.to_csv(name + "_friction.csv")

            # # plotting the desired path
            # x = np.arange(0, len(self.t_meas_), 1)
            # self.u.plot(x, self.y , "desired_traj.png")
            # self.u.plot(x, self.t_meas_ , "torque.png")

        except rospy.ROSInterruptException:
            self.u.stop()


# TODO check the limitations
