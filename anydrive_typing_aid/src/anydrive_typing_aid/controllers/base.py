import rospy
from std_msgs.msg import Empty
import anydrive_typing_aid.utils.utilities as utilities


class BaseController:
    def __init__(self, drv_interface, parameters, save_dir):
        self.drv_interface = drv_interface
        self.parameters = parameters
        self.save_dir = save_dir
        self.log_time_str = utilities.get_time_string()

        self.rate_hz = parameters["rate_hz"]
        self.sampling_time = 1.0 / self.rate_hz

        self.traj_t = None
        self.traj_p = None
        self.traj_v = None
        self.traj_tau = None
        self.lift_start_time = 0.0
        self.lift_running = False

        self.controller_start_time = rospy.get_time()

        log_strings = [
            "t",
            "p_cmd",
            "v_cmd",
            "tau_cmd",
            "p_meas",
            "v_meas",
            "tau_meas",
            "i_meas",
        ]
        self.log = {log_string: list() for log_string in log_strings}

        rospy.Subscriber("lift_arm", Empty, self.lifting_callback)

    def lifting_callback(self, msg):
        if self.lift_running:
            return
        rospy.loginfo("Got triggered")
        self.compute_trajectory()
        self.lift_start_time = rospy.get_time()
        self.lift_running = True

    def stop(self):
        utilities.save_parameters(self)
        utilities.save_log(self)
        self.drv_interface.stop_drive()

    def step(self):
        state = self.drv_interface.get_state()
        res = self.limit_checking(state)
        if not res:
            return False
        current_time = rospy.get_time()
        time_since_lift_start = current_time - self.lift_start_time
        p_cmd = None
        v_cmd = None
        tau_cmd = None
        if time_since_lift_start < self.traj_t[-1]:
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
        self.log["t"].append(current_time - self.controller_start_time)
        self.log["p_cmd"] = p_cmd
        self.log["v_cmd"] = v_cmd
        self.log["tau_cmd"] = tau_cmd
        self.log["p_meas"] = state[0]
        self.log["v_meas"] = state[1]
        self.log["tau_meas"] = state[2]
        self.log["i_meas"] = state[3]
        return True

    def limit_checking(self, state):
        if (
            state[2] < self.parameters["tau_lower_limit"]
            or state[2] > self.parameters["tau_upper_limit"]
        ):
            rospy.logwarn("Torque limit exceeded")
            return False
        return True

    def compute_trajectory(self):
        raise NotImplementedError()
