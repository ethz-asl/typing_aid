import numpy as np
import rospy
from std_msgs.msg import String
import anydrive_typing_aid.utils.utilities as utilities


class BaseController:
    def __init__(self, drv_interface, parameters, save_dir):
        self.drv_interface = drv_interface
        self.parameters = parameters
        self.save_dir = save_dir
        self.log_time_str = utilities.get_time_string()

        self.rate_hz = parameters["rate_hz"]
        self.sampling_time = 1.0 / self.rate_hz

        self.controller_start_time = rospy.get_time()
        self.quit = False

        log_strings = [
            "t",
            "p_cmd",
            "v_cmd",
            "tau_cmd",
            "lift_running",
            "p_meas",
            "v_meas",
            "tau_meas",
            "i_meas",
        ]
        self.log = {log_string: list() for log_string in log_strings}

    def subscribe(self):
        rospy.Subscriber("lift_arm", String, self.lifting_callback_base)

    def lifting_callback(self, msg):
        raise NotImplementedError

    def lifting_callback_base(self, msg):
        if msg.data == "q":
            self.quit = True
        else:
            self.lifting_callback(msg)

    def stop(self):
        self.drv_interface.stop_drive()
        utilities.save_parameters(self)
        utilities.save_log(self)

    def limit_checking(self, state):
        if (
            state[2] < self.parameters["tau_lower_limit"]
            or state[2] > self.parameters["tau_upper_limit"]
        ):
            rospy.logwarn("Torque limit exceeded")
            return False
        return True

    def compute_trajectory(
        self,
        lower_y,
        upper_y,
        duration_up,
        steepness_up,
        duration_down,
        steepness_down,
        duration_const,
        use_depression=False,
        depression_y=None,
        depression_proportion=0.5,
    ):
        if use_depression:
            assert depression_y is not None

        t_ramp_up, y_ramp_up = utilities.sigmoid(
            0.0, duration_up, lower_y, upper_y, steepness_up, self.sampling_time,
        )
        t_const_start = t_ramp_up[-1] + self.sampling_time
        t_const, y_const = utilities.const(
            upper_y, t_const_start, t_const_start + duration_const, self.sampling_time,
        )
        if len(t_const) > 0:
            t_ramp_down_start = t_const[-1] + self.sampling_time
        else:
            t_ramp_down_start = t_const_start

        if not use_depression:
            t_ramp_down, y_ramp_down = utilities.sigmoid(
                t_ramp_down_start,
                t_ramp_down_start + duration_down,
                upper_y,
                lower_y,
                steepness_down,
                self.sampling_time,
            )
        else:
            assert depression_proportion >= 0.0 and depression_proportion <= 1.0
            t_ramp_down_1, y_ramp_down_1 = utilities.sigmoid(
                t_ramp_down_start,
                t_ramp_down_start + depression_proportion * duration_down,
                upper_y,
                depression_y,
                steepness_down,
                self.sampling_time,
            )
            t_ramp_down_2_start = t_ramp_down_1[-1] + self.sampling_time
            t_ramp_down_2, y_ramp_down_2 = utilities.sigmoid(
                t_ramp_down_2_start,
                t_ramp_down_2_start + (1 - depression_proportion) * duration_down,
                depression_y,
                lower_y,
                steepness_down,
                self.sampling_time,
            )
            t_ramp_down = np.concatenate((t_ramp_down_1, t_ramp_down_2))
            y_ramp_down = np.concatenate((y_ramp_down_1, y_ramp_down_2))
        t = np.concatenate((t_ramp_up, t_const, t_ramp_down))
        y = np.concatenate((y_ramp_up, y_const, y_ramp_down))
        dy = utilities.compute_derivative(y, self.sampling_time)
        return t, y, dy

    def step(self):
        state = self.drv_interface.get_state()
        res = self.limit_checking(state)
        if not res or self.quit:
            return False
        current_time = rospy.get_time()

        p_cmd, v_cmd, tau_cmd, lift_running = self.individual_step(current_time, state)

        self.log["t"].append(current_time - self.controller_start_time)
        self.log["p_cmd"].append(p_cmd)
        self.log["v_cmd"].append(v_cmd)
        self.log["tau_cmd"].append(tau_cmd)
        self.log["lift_running"].append(lift_running)
        self.log["p_meas"].append(state[0])
        self.log["v_meas"].append(state[1])
        self.log["tau_meas"].append(state[2])
        self.log["i_meas"].append(state[3])
        return True

    def individual_step(self, current_time, state):
        raise NotImplementedError
