import rospy
from anydrive_typing_aid.controllers.base_controller import BaseController
import anydrive_typing_aid.utils.utilities as utilities


class PositionController(BaseController):
    def __init__(self, drv_interface, rate_hz, save_dir):
        parameters = {
            "idle_torque": 0.5,
            "lifting_mode": utilities.MODE_ID_JOINT_POS,
            "duration_ramp_up": 1.5,
            "distance_ramp_up": 0.6,
            "steepness_ramp_up": 0.05,
            "duration_constant": 1.0,
            "duration_ramp_down": 1.5,
            "steepness_ramp_down": 0.05,
            # "pid_p": 2,
            # "pid_i": 0.078,
            # "pid_d": 0.163,
            "rate_hz": rate_hz,
            "tau_lower_limit": -3.0,
            "tau_upper_limit": 3.0,
        }
        self.traj_t = None
        self.traj_p = None
        self.traj_v = None
        self.traj_tau = None

        self.lift_start_time = 0.0
        self.lift_running = False

        BaseController.__init__(self, drv_interface, parameters, save_dir)
        rospy.loginfo("Controller initialized")

    def lifting_callback(self, msg):
        if self.lift_running:
            return
        rospy.loginfo("Got triggered")
        self.compute_position_trajectory()
        self.lift_start_time = rospy.get_time()
        self.lift_running = True

    def compute_position_trajectory(self):
        p_meas, _, _, _ = self.drv_interface.get_state()
        self.traj_t, self.traj_p, self.traj_v = BaseController.compute_trajectory(
            self,
            lower_y=p_meas,
            upper_y=p_meas + self.parameters["distance_ramp_up"],
            duration_up=self.parameters["duration_ramp_up"],
            steepness_up=self.parameters["steepness_ramp_up"],
            duration_down=self.parameters["duration_ramp_down"],
            steepness_down=self.parameters["steepness_ramp_down"],
            duration_const=self.parameters["duration_constant"],
        )

    def individual_step(self, current_time, state):
        time_since_lift_start = current_time - self.lift_start_time
        p_cmd = None
        v_cmd = None
        tau_cmd = None
        if self.traj_t is not None and time_since_lift_start < self.traj_t[-1]:
            # Interpolate to get command
            idx_next_lower = utilities.find_idx_next_lower(
                self.traj_t, time_since_lift_start
            )
            interpolation_factor = (
                time_since_lift_start - self.traj_t[idx_next_lower]
            ) / (self.traj_t[idx_next_lower + 1] - self.traj_t[idx_next_lower])
            if self.traj_p is not None:
                p_cmd = self.traj_p[idx_next_lower] + interpolation_factor * (
                    self.traj_p[idx_next_lower + 1] - self.traj_p[idx_next_lower]
                )
            if self.traj_v is not None:
                v_cmd = self.traj_v[idx_next_lower] + interpolation_factor * (
                    self.traj_v[idx_next_lower + 1] - self.traj_v[idx_next_lower]
                )
            if self.traj_tau is not None:
                tau_cmd = self.traj_tau[idx_next_lower] + interpolation_factor * (
                    self.traj_tau[idx_next_lower + 1] - self.traj_tau[idx_next_lower]
                )
            self.drv_interface.move(
                self.parameters["lifting_mode"],
                position=p_cmd,
                velocity=v_cmd,
                torque=tau_cmd,
                params=self.parameters,
            )
        else:
            tau_cmd = self.parameters["idle_torque"]
            self.drv_interface.move(utilities.MODE_ID_JOINT_TRQ, torque=tau_cmd)
            self.lift_running = False
        return p_cmd, v_cmd, tau_cmd, self.lift_running
