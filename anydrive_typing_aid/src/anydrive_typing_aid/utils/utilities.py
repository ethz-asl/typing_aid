import rospy
from std_msgs.msg import String, Float64, Header, Empty
from math import pi
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime
import csv
import os

MODE_ID_FREEZE = 1
MODE_ID_JOINT_POS = 8
MODE_ID_JOINT_VEL = 9
MODE_ID_JOINT_TRQ = 10
MODE_ID_JOINT_POS_VEL = 11
MODE_ID_JOINT_POS_VEL_TRQ = 12


def state_translator(state_int):
    state_strings = [
        "NA",
        "Calibrate",
        "ColdStart",
        "Configure",
        "ControlOp",
        "DeviceMissing",
        "Error",
        "Fatal",
        "MotorOp",
        "MotorPreOp",
        "Standby",
        "WarmStart",
    ]
    return state_strings[state_int]


def logistic_fct(x, midpoint, max_val, steepness):
    res = max_val / (1 + np.exp(-steepness * (x - midpoint)))
    return res


def quadratic_fct(t0, t_end, tau_0, tau_end, sampling_time):
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


def polynomial5(t0, t2, y0, y2, steepness, sampling_time):
    t1 = (t2 - t0) / 2.0 + t0
    y1 = (y2 - y0) / 2.0 + y0
    steepness = steepness * (y2 - y0) / (t2 - t0)
    a = np.array(
        [
            [1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
            [1, t1, t1 ** 2, t1 ** 3, t1 ** 4, t1 ** 5],
            [1, t2, t2 ** 2, t2 ** 3, t2 ** 4, t2 ** 5],
            [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
            [0, 1, 2 * t1, 3 * t1 ** 2, 4 * t1 ** 3, 5 * t1 ** 4],
            [0, 1, 2 * t2, 3 * t2 ** 2, 4 * t2 ** 3, 5 * t2 ** 4],
        ]
    )
    b = np.array([y0, y1, y2, 0, steepness, 0])
    p = np.linalg.solve(a, b)
    f = (
        lambda t: p[0]
        + p[1] * t
        + p[2] * t ** 2
        + p[3] * t ** 3
        + p[4] * t ** 4
        + p[5] * t ** 5
    )
    f_d = (
        lambda t: p[1]
        + 2 * p[2] * t
        + 3 * p[3] * t ** 2
        + 4 * p[4] * t ** 3
        + 5 * p[5] * t ** 4
    )
    f_dd = lambda t: 2 * p[2] + 6 * p[3] * t + 12 * p[4] * t ** 2 + 20 * p[5] * t ** 3
    t = np.linspace(t0, t2, np.ceil((t2 - t0) / sampling_time))
    y = map(f, t)
    y_d = map(f_d, t)
    y_dd = map(f_dd, t)
    return t, y, y_d, y_dd


def sigmoid(t0, t1, y0, y1, steepness, sampling_time):
    t_center = t0 + (t1 - t0) / 2
    f = lambda t: y0 + (y1 - y0) * (
        1 - 1 / (1 + np.exp(-(t_center - t) / (steepness * (t1 - t0))))
    )
    t = np.arange(t0, t1, sampling_time)
    y = map(f, t)
    return t, y


def const(value, t0, t_end, sampling_time):
    t = np.arange(t0, t_end, sampling_time)
    y = value * np.ones(len(t))
    return t, y


def afine(a, tau_0, t0, sampling_time):
    duration = 50
    num_samples = round(float(duration) / float(sampling_time))
    x = np.linspace(t0, t0 + 50, num_samples, endpoint=True)
    y = a * x + tau_0
    return x, y


# put the pieces together
def torque_profile(y1, y2, y3, x1, x2, x3):
    x = np.concatenate((x1, x2, x3))
    y = np.concatenate((y1, y2, y3))
    return x, y


def add_profile(x4, y4, x5, y5):
    x = np.concatenate((x4, x5))
    y = np.concatenate((y4, y5))
    return x, y


def store(t_meas, v_meas, p_meas, i_meas, t_meas_, v_meas_, p_meas_, i_meas_):
    t_meas_.append(t_meas)
    v_meas_.append(v_meas)
    p_meas_.append(p_meas)
    i_meas_.append(i_meas)
    return t_meas_, v_meas_, p_meas_, i_meas_


def store_pid(t_meas, v_meas, p_meas, t_meas_, v_meas_, p_meas_):
    t_meas_.append(t_meas)
    v_meas_.append(v_meas)
    p_meas_.append(p_meas)
    return t_meas_, v_meas_, p_meas_


def store_one(var, array):
    array.append(var)
    return array


def plot(x, y, title):
    plt.figure()
    plt.plot(x, y)
    plt.show()
    # plt.savefig(get_time_string() + title)


def check_sign(t_meas, t_next):
    if (t_meas - t_next) > 0:
        return 1
    else:
        return -1


def get_time_string():
    date = datetime.now()
    name = date.strftime("%Y-%m-%d_%H-%M-%S")
    return name


def concat_data(
    t_cmd, name_t_cmd, t_meas_, v_meas_, p_meas_, i_meas_, name_file, folder
):
    name = get_time_string()
    path = "/home/asl-admin/Desktop/" + folder
    data_concat = np.array(
        (t_cmd[: len(t_meas_)], t_meas_, v_meas_, p_meas_, i_meas_)
    ).T

    data_pd = pd.DataFrame(
        data=data_concat,
        columns=[name_t_cmd, "torque", "velocity", "position", "current"],
    )
    data_pd.to_csv(path + name + name_file + ".csv")


def create_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def save_parameters(controller):
    name = controller.log_time_str + "_" + controller.__class__.__name__ + "_params.txt"
    with open(os.path.join(controller.save_dir, name), "w") as f:
        f.write(str(controller.parameters))


def save_log(controller):
    name = controller.log_time_str + "_" + controller.__class__.__name__ + "_log.csv"
    df = pd.DataFrame(controller.log)
    df.to_csv(os.path.join(controller.save_dir, name))
    rospy.loginfo("Stored data under {}.".format(name))


def compute_traj(
    param, param_0, other_start_val, param_end, param_low, other_traj, sampling_time,
):
    if other_start_val == -99:
        x1, y1 = quadratic_fct(
            0.0,
            param["transition_up"],
            param[param_0],
            param[param_end],
            sampling_time,
        )
    else:
        x1, y1 = quadratic_fct(
            0.0,
            param["transition_up"],
            other_start_val,
            param[param_end],
            sampling_time,
        )
    x2, y2 = const(
        param[param_end],
        param["transition_up"],
        param["transition_up"] + param["duration_constant_up"],
        sampling_time,
    )
    if not other_traj:
        x3, y3 = quadratic_fct(
            param["transition_up"] + param["duration_constant_up"],
            param["transition_up"]
            + param["duration_constant_up"]
            + param["transition_down"],
            param[param_end],
            param[param_0],
            sampling_time,
        )
        # put everything together
        x, y = torque_profile(y1, y2, y3, x1, x2, x3)
    else:
        half_time = (
            param["transition_down"] / 2.0
            + param["transition_up"]
            + param["duration_constant_up"]
        )
        x3, y3 = quadratic_fct(
            param["transition_up"] + param["duration_constant_up"],
            half_time,
            param[param_end],
            param[param_low],
            sampling_time,
        )
        x4, y4 = quadratic_fct(
            half_time,
            param["transition_up"]
            + param["duration_constant_up"]
            + param["transition_down"],
            param[param_low],
            param[param_0],
            sampling_time,
        )
        x5, y5 = torque_profile(y1, y2, y3, x1, x2, x3)
        x, y = add_profile(x5, y5, x4, y4)
    return x, y


def compute_derivative(y, sampling_time):
    y_ext = np.concatenate(([y[0], y[0]], y, [y[-1], y[-1]]))
    dy = np.zeros(len(y))
    # dy_simple = np.zeros(len(y))
    for i in range(len(y)):
        i_ext = i + 2
        dy[i] = (
            -y_ext[i_ext + 2]
            + 8 * y_ext[i_ext + 1]
            - 8 * y_ext[i_ext - 1]
            + y_ext[i_ext - 2]
        ) / (12 * sampling_time)
        # dy_simple[i] = (y_ext[i_ext + 1] - y_ext[i_ext - 1]) / (2 * sampling_time)
    return dy


def find_idx_next_lower(array, value):
    idx = (np.abs(array - value)).argmin()
    if array[idx] > value:
        idx -= 1
    return idx
