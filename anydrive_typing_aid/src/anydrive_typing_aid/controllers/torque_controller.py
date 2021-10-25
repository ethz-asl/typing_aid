import rospy
import matplotlib.pyplot as plt

from anydrive_typing_aid.controllers.base_controller import BaseController
import anydrive_typing_aid.utils.utilities as utilities


class TorqueController(BaseController):
    def __init__(self, drv_interface, rate_hz, save_dir):
        parameters = {
            "idle_torque": 0.7,
            "duration_ramp_up": 0.7,
            "distance_ramp_up": 1.2,
            "steepness_ramp_up": 0.075,
            "duration_constant": 0.1,
            "duration_ramp_down": 0.5,
            "steepness_ramp_down": 0.075,
            "use_depression": True,
            "distance_depression": -0.3,
            "depression_proportion": 0.5,
            # "pid_p": 2,
            # "pid_i": 0.078,
            # "pid_d": 0.163,
            "rate_hz": rate_hz,
            "tau_lower_limit": -3.0,
            "tau_upper_limit": 3.0,
        }
        BaseController.__init__(self, drv_interface, parameters, save_dir)

        self.traj_t, self.traj_tau, _ = BaseController.compute_trajectory(
            self,
            lower_y=self.parameters["idle_torque"],
            upper_y=self.parameters["idle_torque"]
            + self.parameters["distance_ramp_up"],
            duration_up=self.parameters["duration_ramp_up"],
            steepness_up=self.parameters["steepness_ramp_up"],
            duration_down=self.parameters["duration_ramp_down"],
            steepness_down=self.parameters["steepness_ramp_down"],
            duration_const=self.parameters["duration_constant"],
            use_depression=self.parameters["use_depression"],
            depression_y=self.parameters["idle_torque"]
            + self.parameters["distance_depression"],
            depression_proportion=self.parameters["depression_proportion"],
        )

        self.lift_start_time = 0.0
        self.lift_running = False

        rospy.loginfo("Controller initialized")

    def plot_trajectory(self):
        plt.plot(self.traj_t, self.traj_tau)
        plt.legend(["torque"])
        plt.show()

    def lifting_callback(self, msg):
        # if self.lift_running:
        #     return
        rospy.loginfo("Got triggered")
        self.lift_start_time = rospy.get_time()
        self.lift_running = True

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
            tau_cmd = self.traj_tau[idx_next_lower] + interpolation_factor * (
                self.traj_tau[idx_next_lower + 1] - self.traj_tau[idx_next_lower]
            )
        else:
            tau_cmd = self.parameters["idle_torque"]
            self.lift_running = False
        self.drv_interface.move(utilities.MODE_ID_JOINT_TRQ, torque=tau_cmd)
        return p_cmd, v_cmd, tau_cmd, self.lift_running
