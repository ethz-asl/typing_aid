import rospy
import numpy as np

import anydrive_typing_aid.utils.utilities as utilities
from anydrive_typing_aid.controllers.base_controller import BaseController


class FrictionController(BaseController):
    def __init__(self, drv_interface, rate_hz, save_dir):
        parameters = {
            "idle_torque": 0.5,
            "delta_tau_up_per_sec": 1.2,
            "delta_tau_down_per_sec": 1.2,
            "upper_torque_limit": 2.0,
            "velocity_horizon": 0.5,
            "velocity_limit": 4.0,
            # "desired_up_duration": 0.5,
            # "desired_up_tolerance": 0.1,
            # "pid_p": 2,
            # "pid_i": 0.078,
            # "pid_d": 0.163,
            "rate_hz": rate_hz,
            "tau_lower_limit": -3.0,
            "tau_upper_limit": 3.0,
        }

        # self.param = {
        #     "tau_0": 0.5,
        #     "tau_up": 2.0,
        #     "v_des": 4,
        #     "d_tau_up_per_sec": 1.2,
        #     "d_tau_down_per_sec": 1.2,
        #     "tau_min": -1.0,
        #     "tau_max": 3.0,
        #     "x_0_lim": -3.8042964935302734,
        #     "x_end_lim": 4.61220121383667,
        #     "avg_time_vel": 0.5,
        #     "avg_time_up": 1.0,
        # }
        self.d_tau_up = parameters["delta_tau_up_per_sec"] / parameters["rate_hz"]
        self.d_tau_down = parameters["delta_tau_down_per_sec"] / parameters["rate_hz"]

        self.current_torque = parameters["idle_torque"]

        # self.avg_num_iter = int(round(self.param["avg_time_vel"] * rate_hz))
        # self.desired_up_timesteps = int(
        #     round(self.param["desired_up_duration"] * rate_hz)
        # )
        self.velocity_buffer_timesteps = int(
            np.round(parameters["velocity_horizon"] * rate_hz)
        )
        self.velocity_buffer = None

        self.lift_running = False
        self.moving_up = False
        self.idx = 0

        BaseController.__init__(self, drv_interface, parameters, save_dir)
        rospy.loginfo("Controller initialized")

    # def check_velocity(self, v_meas, v_des):
    #     if self.avg_vel():
    #         if self.avg_v >= v_des:
    #             return True
    #         else:
    #             return False
    #     else:
    #         return False
    #
    # def avg_vel(self):
    #     if len(self.v_meas_) < self.avg_num_iter:
    #         return False
    #     else:
    #         self.avg_v = mean(self.v_meas_[-self.avg_num_iter :])
    #         return True

    def lifting_callback(self, msg):
        if self.lift_running:
            return
        self.idx = 0
        self.velocity_buffer = [0.0] * self.velocity_buffer_timesteps
        rospy.loginfo("Got triggered")
        self.lift_running = True
        self.moving_up = True

    def individual_step(self, current_time, state):
        self.velocity_buffer[self.idx] = state[1]
        self.idx = (self.idx + 1) % self.velocity_buffer_timesteps
        p_cmd = None
        v_cmd = None
        if self.lift_running:
            if self.moving_up:
                self.current_torque += self.d_tau_up
                if self.current_torque > self.parameters["upper_torque_limit"]:
                    self.current_torque = self.parameters["upper_torque_limit"]
                    self.moving_up = False
                    rospy.loginfo("Finished moving up")
                elif np.mean(self.velocity_buffer) > self.parameters["velocity_limit"]:
                    self.moving_up = False
                    rospy.loginfo("Finished moving up")
            else:
                self.current_torque -= self.d_tau_down
                if self.current_torque < self.parameters["idle_torque"]:
                    self.current_torque = self.parameters["idle_torque"]
                    self.lift_running = False
        tau_cmd = self.current_torque
        self.drv_interface.move(utilities.MODE_ID_JOINT_TRQ, torque=tau_cmd)
        return p_cmd, v_cmd, tau_cmd, self.lift_running

    # def run(self):
    #     rospy.loginfo("starting movement")
    #     try:
    #         t_meas, v_meas, p_meas, _ = self.u.listener()
    #         while not rospy.is_shutdown():
    #             if self.move_up:
    #                 self.current_torque += self.d_tau_up
    #                 if self.current_torque > self.param["tau_up"]:
    #                     self.current_torque = self.param["tau_up"]
    #                     if (
    #                         len(self.t_meas_) > self.num_tau_up
    #                         and mean(self.t_meas_[-self.num_tau_up :])
    #                         >= self.param["tau_up"] - 0.1
    #                     ):
    #                         self.move_up = False
    #                         print("Finished moving up")
    #                 if self.check_velocity(v_meas, self.param["v_des"]):
    #                     self.move_up = False
    #                     print("Finished moving up")
    #             else:
    #                 self.current_torque -= self.d_tau_down
    #                 if self.current_torque < self.param["tau_0"]:
    #                     self.current_torque = self.param["tau_0"]
    #
    #             if self.u.lim_check(self.param, self.t_meas_, p_meas):
    #                 raise rospy.ROSInterruptException
    #
    #             self.u.move(JOINT_TORQUE, self.p_des, self.v_des, self.current_torque)
    #             self.t_cmd_ = self.u.store_one(self.current_torque, self.t_cmd_)
    #             t_meas, v_meas, p_meas, i_meas = self.u.listener()
    #             self.t_meas_, self.v_meas_, self.p_meas_, self.i_meas_ = self.u.store(
    #                 t_meas,
    #                 v_meas,
    #                 p_meas,
    #                 i_meas,
    #                 self.t_meas_,
    #                 self.v_meas_,
    #                 self.p_meas_,
    #                 self.i_meas_,
    #             )
    #             # print("Current torque: {}".format(self.current_torque))
    #             self.rate.sleep()
    #
    #     except rospy.ROSInterruptException:
    #         self.stop()
