import rospy
import numpy as np

import anydrive_typing_aid.utils.utilities as utilities
from anydrive_typing_aid.controllers.base_controller import BaseController


class ImpedanceController(BaseController):
    def __init__(self, drv_interface, rate_hz, save_dir):
        parameters = {
            "impedance_gain": 0.7,
            "idle_torque": 0.5,
            "duration_ramp_up": 0.6,
            "distance_ramp_up": 2.0,
            "steepness_ramp_up": 0.05,
            "duration_constant": 0.1,
            "duration_ramp_down": 0.6,
            "steepness_ramp_down": 0.05,
            "use_depression": True,
            "distance_depression": -0.2,
            # "pid_p": 2,
            # "pid_i": 0.078,
            # "pid_d": 0.163,
            "rate_hz": rate_hz,
            "tau_lower_limit": -3.0,
            "tau_upper_limit": 3.0,
        }

        self.traj_t = None
        self.traj_p = None

        self.lift_start_time = 0.0
        self.lift_running = False

        BaseController.__init__(self, drv_interface, parameters, save_dir)

    def lifting_callback(self, msg):
        if self.lift_running:
            return
        rospy.loginfo("Got triggered")
        self.compute_position_trajectory()
        self.lift_start_time = rospy.get_time()
        self.lift_running = True

    def compute_position_trajectory(self):
        p_meas, _, _, _ = self.drv_interface.listener()
        self.traj_t, self.traj_p, _ = BaseController.compute_trajectory(
            self,
            lower_y=p_meas,
            upper_y=p_meas + self.parameters["distance_ramp_up"],
            duration_up=self.parameters["duration_ramp_up"],
            steepness_up=self.parameters["steepness_ramp_up"],
            duration_down=self.parameters["duration_ramp_down"],
            steepness_down=self.parameters["steepness_ramp_down"],
            duration_const=self.parameters["duration_constant"],
            use_depression=self.parameters["use_depression"],
            depression_y=p_meas + self.parameters["distance_depression"],
        )

    def individual_step(self, current_time, state):
        time_since_lift_start = current_time - self.lift_start_time
        p_cmd = None
        v_cmd = None
        if time_since_lift_start < self.traj_t[-1]:
            # Interpolate to get command
            idx_next_lower = utilities.find_idx_next_lower(
                self.traj_t, time_since_lift_start
            )
            interpolation_factor = (
                time_since_lift_start - self.traj_t[idx_next_lower]
            ) / (self.traj_t[idx_next_lower + 1] - self.traj_t[idx_next_lower])
            p_desired = self.traj_p[idx_next_lower] + interpolation_factor * (
                self.traj_p[idx_next_lower + 1] - self.traj_p[idx_next_lower]
            )
            tau_cmd = self.parameters["impedance_gain"] * (p_desired - state[0])
            tau_cmd = np.max((tau_cmd, self.parameters["idle_torque"]))
        else:
            tau_cmd = self.parameters["idle_torque"]
            self.lift_running = False
        self.drv_interface.move(utilities.MODE_ID_JOINT_TRQ, torque=tau_cmd)
        return p_cmd, v_cmd, tau_cmd, self.lift_running
