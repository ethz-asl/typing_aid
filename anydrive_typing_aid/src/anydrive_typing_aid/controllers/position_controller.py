import rospy
from std_msgs.msg import String, Float64, Header, Empty
import pandas as pd
import numpy as np

from anydrive_typing_aid.controllers.base import BaseController


class PositionController(BaseController):
    def __init__(self, drv_interface):
        BaseController.__init__(self, drv_interface)

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
        self.u = utils.utils()
        self.parameters = {
            "transition_up": 0.6,
            "duration_constant_up": 0.0,
            "transition_down": 0.5,
            "x_end": 0.4290352761745453,
            "x_0_lim": -5.911866188049316,
            "x_end_lim": 4.441066265106201,
            "x_0": -4.50688362121582,
            "rate": 200,
            "tau_0": 0.5,
            "p": 2,
            "i": 0.078,
            "d": 0.163,
            "tau_min": -1.0,
            "tau_max": 3.0,
        }

        # self.param = self.u.set_pos(self.param)
        # print(self.param)
        self.u.save_param(self.param, "pos_control", "pos_control/")
        self.rate_hz = self.param["rate"]
        self.sampling_time = 1.0 / self.rate_hz

        self.rate = rospy.Rate(self.rate_hz)

        rospy.Subscriber("lift_arm", Empty, self.callback)
        rospy.loginfo("Controller init finished")

        self.other_traj = False
        self.steps_left = 0

    def callback(self, msg):
        if self.steps_left > 0:
            return
        rospy.loginfo("Got triggered")
        _, _, p_meas, _ = self.u.listener()
        self.x, self.y = self.u.compute_traj(
            self.param, "x_0", p_meas, "x_end", _, self.other_traj, self.sampling_time,
        )
        self.dy = self.u.compute_der(self.y, self.rate_hz)
        # self.u.plot(self.x, self.y, "desired_traj.png")
        # self.u.plot(self.x[:-1], self.dy, "desired_vel.png")
        self.total_steps = len(self.y)
        self.steps_left = self.total_steps

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
        name = self.u.get_time()
        path = "/home/asl-admin/Desktop/pos_control/"
        data_pd.to_csv(path + name + "_pos_control.csv")
        # # plotting the desired path
        # x = np.arange(0, len(self.t_meas_), 1)
        # self.u.plot(x, self.y , "desired_traj.png")
        # self.u.plot(x, self.t_meas_ , "torque.png")
        self.u.stop_drive()

    def run(self):
        rospy.loginfo("Starting position controller")
        try:
            # change pid gains
            self.u.pid(JOINT_POSITION, self.param)
            # also dn work when unchanged
            while not rospy.is_shutdown():
                if self.steps_left > 0:
                    # Moving up
                    self.p_cmd = self.y[self.total_steps - self.steps_left]
                    self.v_cmd = self.dy[self.total_steps - self.steps_left]
                    self.u.move(JOINT_POSITION, self.p_cmd, self.v_cmd, self.t_cmd)
                    self.t_cmd = 0.0
                    self.steps_left -= 1
                else:
                    self.t_cmd = self.param["tau_0"]
                    self.u.move(JOINT_TORQUE, self.p_cmd, self.v_cmd, self.t_cmd)
                    self.p_cmd, self.v_cmd = 0.0, 0.0

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
                self.t_cmd_, self.v_cmd_, self.p_cmd_ = self.u.store_pid(
                    self.t_cmd,
                    self.v_cmd,
                    self.p_cmd,
                    self.t_cmd_,
                    self.v_cmd_,
                    self.p_cmd_,
                )
                if self.u.lim_check(self.param, self.t_meas_, p_meas):
                    raise rospy.ROSInterruptException
                self.rate.sleep()

        except rospy.ROSInterruptException:
            self.stop()
