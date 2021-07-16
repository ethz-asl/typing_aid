import numpy as np
from anydrive_typing_aid.controllers.base import BaseController
import anydrive_typing_aid.utils.utilities as utilities


class PositionController(BaseController):
    def __init__(self, drv_interface, rate_hz, save_dir):
        parameters = {
            "idle_torque": 0.5,
            "lifting_mode": utilities.MODE_ID_JOINT_POS_VEL,
            "duration_ramp_up": 0.6,
            "distance_ramp_up": 2.0,
            "steepness_ramp_up": 0.05,
            "duration_constant": 0.1,
            "duration_ramp_down": 0.6,
            "steepness_ramp_down": 0.05,
            # "pid_p": 2,
            # "pid_i": 0.078,
            # "pid_d": 0.163,
            "rate_hz": rate_hz,
            "tau_lower_limit": -3.0,
            "tau_upper_limit": 3.0,
        }
        BaseController.__init__(self, drv_interface, parameters, save_dir)

    def compute_trajectory(self):
        p_meas, _, _, _ = self.drv_interface.listener()
        t_ramp_up, y_ramp_up = utilities.sigmoid(
            0.0,
            self.parameters["duration_ramp_up"],
            p_meas,
            p_meas + self.parameters["distance_ramp_up"],
            self.parameters["steepness_ramp_up"],
            self.sampling_time,
        )
        t_const_start = t_ramp_up[-1] + self.sampling_time
        t_const, y_const = utilities.const(
            p_meas + self.parameters["distance_ramp_up"],
            t_const_start,
            t_const_start + self.parameters["duration_constant"],
            self.sampling_time,
        )
        if len(t_const) > 0:
            t_ramp_down_start = t_const[-1] + self.sampling_time
        else:
            t_ramp_down_start = t_const_start
        t_ramp_down, y_ramp_down = utilities.sigmoid(
            t_ramp_down_start,
            t_ramp_down_start + self.parameters["duration_ramp_down"],
            p_meas + self.parameters["distance_ramp_up"],
            p_meas,
            self.parameters["steepness_ramp_down"],
            self.sampling_time,
        )
        t = np.concatenate((t_ramp_up, t_const, t_ramp_down))
        y = np.concatenate((y_ramp_up, y_const, y_ramp_down))
        dy = utilities.compute_derivative(y, self.sampling_time)
        self.traj_t = t
        self.traj_p = y
        self.traj_v = dy
