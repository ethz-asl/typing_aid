# This is a bang band position controller, changing the position
# setpoint abruptly when the trigger is received. Not worth
# further pursuing.

import rospy
from std_msgs.msg import String, Float64, Header, Empty
import anydrive_msgs.msg as msg_defs
from math import pi
import pandas as pd
import numpy as np

import fsmstate as fsm
import utils

# global JOINT_POSITION, JOINT_VELOCITY, JOINT_TORQUE, JOINT_POSITION_VELOCITY
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE = 10
JOINT_POSITION_VELOCITY = 11


class Basic_controller:
    def __init__(self):

        self.p_cmd, self.v_cmd, self.t_cmd = 0, 0, 0
        (
            self.t_meas_,
            self.v_meas_,
            self.p_meas_,
            self.i_meas_,
            self.dy,
            self.t_cmd_,
            self.v_cmd_,
            self.p_cmd_,
        ) = ([], [], [], [], [], [], [], [])
        self.utils = utils.utils()
        self.param = {
            "x_0_lim": -13.305917739868164,
            "x_end_lim": -1.3176895380020142,
            "x_0": -11.7498140335083,
            "x_end": -6.833189487457275,
            "tau_0": 0.5,
            "tau_min": -1.0,
            "tau_max": 3.0,
            "rate": 1,
        }

        # self.param = self.utils.set_pos(self.param)
        # print(self.param)
        self.utils.save_param(self.param, "pos_control", "pos_control/")
        self.cmd_store = 0.0

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")
        rate_hz = self.param["rate"]
        self.rate = rospy.Rate(rate_hz)
        self.steps_left = 0
        self.i = 1

    def callback(self, msg):
        self.p_cmd = self.param["x_end"]
        self.steps_left = 2

    def stop(self):
        print("Exit handler")
        # saving the data
        data_concat = np.array(
            (
                self.t_cmd_[: len(self.t_meas_)],
                self.v_cmd_[: len(self.t_meas_)],
                self.p_cmd_[: len(self.t_meas_)],
                self.t_meas_,
                self.v_meas_,
                self.p_meas_,
                self.i_meas_,
            )
        ).T
        data_pd = pd.DataFrame(
            data=data_concat,
            columns=[
                "commanded torque",
                "commanded velocity",
                "commanded position",
                "torque",
                "velovity",
                "position",
                "current",
            ],
        )
        name = self.utils.get_time()
        path = "/home/asl-admin/Desktop/basic/"
        data_pd.to_csv(path + name + "_basic_pos_control.csv")
        # # plotting the desired path
        # x = np.arange(0, len(self.t_meas_), 1)
        # self.utils.plot(x, self.y , "desired_traj.png")
        # self.utils.plot(x, self.t_meas_ , "torque.png")
        self.utils.stop_drive()

    def run(self):
        rospy.loginfo("starting movement")
        try:

            # change pid gains
            # self.utils.pid(JOINT_POSITION, self.param)
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    self.utils.move(JOINT_POSITION, self.p_cmd, self.v_cmd, self.t_cmd)
                    self.cmd_store = self.p_cmd
                    self.t_cmd = 0.0
                    self.steps_left -= 1
                    self.p_cmd = self.param["x_0"]
                else:
                    self.t_cmd = self.param["tau_0"]
                    self.utils.move(JOINT_TORQUE, self.p_cmd, self.v_cmd, self.t_cmd)
                    self.p_cmd, self.v_cmd, self.cmd_store = 0.0, 0.0, 0.0

                t_meas, v_meas, p_meas, i_meas = self.utils.listener()
                (
                    self.t_meas_,
                    self.v_meas_,
                    self.p_meas_,
                    self.i_meas_,
                ) = self.utils.store(
                    t_meas,
                    v_meas,
                    p_meas,
                    i_meas,
                    self.t_meas_,
                    self.v_meas_,
                    self.p_meas_,
                    self.i_meas_,
                )
                self.t_cmd_, self.v_cmd_, self.p_cmd_ = self.utils.store_pid(
                    self.t_cmd,
                    self.v_cmd,
                    self.p_cmd,
                    self.t_cmd_,
                    self.v_cmd_,
                    self.p_cmd_,
                )
                if self.utils.lim_check(self.param, self.t_meas_, p_meas):
                    raise rospy.ROSInterruptException
                self.rate.sleep()

        except rospy.ROSInterruptException:
            self.stop()
