#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import csv
from statistics import mean

import fsmstate as fsm
import pid


global JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE = 10


class Utilities:
    def __init__(self):
        pass

    def logistic_fct(self, x, midpoint, max_val, steepness):
        res = max_val / (1 + np.exp(-steepness * (x - midpoint)))
        return res

    # used to compute the point for the quadratic fct
    # set const to zero to begin
    def quadratic_fct(self, t0, t_end, tau_0, tau_end, sampling_time):
        t_des = float(t_end - t0) / 2.0 + t0
        A = np.array(
            [
                [2 * t0, 1, 0, 0, 0, 0],
                [0, 0, 0, 2 * t_end, 1, 0],
                [t_des ** 2, t_des, 1, -(t_des ** 2), -t_des, -1],
                [2 * t_des, 1, 0, -2 * t_des, -1, 0],
                [t0 ** 2, t0, 1, 0, 0, 0],
                [0, 0, 0, t_end ** 2, t_end, 1],
            ]
        )
        b = np.array([0, 0, 0, 0, tau_0, tau_end])
        b = b.transpose()
        c = np.linalg.inv(A)
        c = c.dot(b)

        duration = t_end - t0
        num_samples = round(float(duration) / float(sampling_time))
        half_num_samples = int(num_samples / 2.0)

        x1 = np.linspace(t0, t_des, half_num_samples, endpoint=True)
        y1 = c[0] * x1 ** 2 + c[1] * x1 + c[2]

        x2 = np.linspace(t_des, t_end, half_num_samples, endpoint=True)
        y2 = c[3] * x2 ** 2 + c[4] * x2 + c[5]

        x = np.concatenate((x1[:-1], x2))
        y = np.concatenate((y1[:-1], y2))
        return x, y

    # to generate straight line. Duration in s.
    def const(self, tau, t0, t_end, sampling_time):
        duration = t_end - t0
        num_samples = round(float(duration) / float(sampling_time))
        x = np.linspace(t0, t_end, num_samples, endpoint=True)
        y = tau * np.ones(int(num_samples))
        return x, y

    def afine(self, a, tau_0, t0, sampling_time):
        duration = 50
        num_samples = round(float(duration) / float(sampling_time))
        x = np.linspace(t0, t0 + 50, num_samples, endpoint=True)
        y = a * x + tau_0
        return x, y

    # put the pieces together
    def torque_profile(self, y1, y2, y3, x1, x2, x3):
        x = np.concatenate((x1, x2, x3))
        y = np.concatenate((y1, y2, y3))
        return x, y

    def add_profile(self, x4, y4, x5, y5):
        x = np.concatenate((x4, x5))
        y = np.concatenate((y4, y5))
        return x, y

    def store(self, t_meas, v_meas, p_meas, i_meas, t_meas_, v_meas_, p_meas_, i_meas_):
        t_meas_.append(t_meas)
        v_meas_.append(v_meas)
        p_meas_.append(p_meas)
        i_meas_.append(i_meas)
        return t_meas_, v_meas_, p_meas_, i_meas_

    def store_pid(self, t_meas, v_meas, p_meas, t_meas_, v_meas_, p_meas_):
        t_meas_.append(t_meas)
        v_meas_.append(v_meas)
        p_meas_.append(p_meas)
        return t_meas_, v_meas_, p_meas_

    def store_one(self, var, array):
        array.append(var)
        return array

    def plot(self, x, y, title):
        plt.figure()
        plt.plot(x, y)
        plt.show()
        # plt.savefig(self.get_time() + title)

    def check_sign(self, t_meas, t_next):
        if (t_meas - t_next) > 0:
            return 1
        else:
            return -1

    def get_time(self):
        date = datetime.now()
        name = date.strftime("%Y-%m-%d_%H-%M-%S")
        return name

    def concat_data(
        self, t_cmd, name_t_cmd, t_meas_, v_meas_, p_meas_, i_meas_, name_file, folder
    ):
        name = self.get_time()
        path = "/home/asl-admin/Desktop/" + folder
        data_concat = np.array(
            (t_cmd[: len(t_meas_)], t_meas_, v_meas_, p_meas_, i_meas_)
        ).T

        data_pd = pd.DataFrame(
            data=data_concat,
            columns=[name_t_cmd, "torque", "velocity", "position", "current"],
        )
        data_pd.to_csv(path + name + name_file + ".csv")

    def save_param(self, param, controller, folder):
        name = self.get_time() + controller + "_param.csv"
        with open("/home/asl-admin/Desktop/" + folder + name, "w") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=list(param.keys()))
            writer.writeheader()
            writer.writerows([param])

    def compute_traj(
        self,
        param,
        param_0,
        other_start_val,
        param_end,
        param_low,
        other_traj,
        sampling_time,
    ):
        if other_start_val == -99:
            x1, y1 = self.quadratic_fct(
                0.0,
                param["transition_up"],
                param[param_0],
                param[param_end],
                sampling_time,
            )
        else:
            x1, y1 = self.quadratic_fct(
                0.0,
                param["transition_up"],
                other_start_val,
                param[param_end],
                sampling_time,
            )
        x2, y2 = self.const(
            param[param_end],
            param["transition_up"],
            param["transition_up"] + param["duration_constant_up"],
            sampling_time,
        )
        if not other_traj:
            x3, y3 = self.quadratic_fct(
                param["transition_up"] + param["duration_constant_up"],
                param["transition_up"]
                + param["duration_constant_up"]
                + param["transition_down"],
                param[param_end],
                param[param_0],
                sampling_time,
            )
            # put everything together
            x, y = self.torque_profile(y1, y2, y3, x1, x2, x3)
        else:
            half_time = (
                param["transition_down"] / 2.0
                + param["transition_up"]
                + param["duration_constant_up"]
            )
            x3, y3 = self.quadratic_fct(
                param["transition_up"] + param["duration_constant_up"],
                half_time,
                param[param_end],
                param[param_low],
                sampling_time,
            )
            x4, y4 = self.quadratic_fct(
                half_time,
                param["transition_up"]
                + param["duration_constant_up"]
                + param["transition_down"],
                param[param_low],
                param[param_0],
                sampling_time,
            )
            x5, y5 = self.torque_profile(y1, y2, y3, x1, x2, x3)
            x, y = self.add_profile(x5, y5, x4, y4)
        return x, y

    def compute_der(self, x, rate):
        dx = np.diff(x) * rate
        dx = np.append(dx, dx[-1])
        return dx
