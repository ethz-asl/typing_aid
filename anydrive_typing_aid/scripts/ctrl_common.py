import utils

import rospy


class BaseController(object):
    def __init__(self):
        print("Initialize base controller")

        self.u = utils.utils()

        self.rate_hz = 20
        self.sampling_time = 1.0 / self.rate_hz

        self.rate = rospy.Rate(self.rate_hz)

        self.param = None

    # transition time is the time needed to go from low pos to high pos
    # x_0 is low torque value and x_end is high torque value
    def compute_traj(self, p_meas):
        params = self.param
        # way up :
        x1, y1 = self.u.quadratic_fct(
            params["t0"],
            params["t0"] + params["transition"],
            p_meas,
            params["x_end"],
            self.sampling_time,
        )
        x2, y2 = self.u.const(
            params["x_end"],
            params["t0"] + params["transition"],
            params["t_end"] - params["transition"],
            self.sampling_time,
        )
        x3, y3 = self.u.quadratic_fct(
            params["t_end"] - params["transition"],
            params["t_end"],
            params["x_end"],
            params["x_0"],
            self.sampling_time,
        )
        # put everything together
        return self.u.torque_profile(y1, y2, y3, x1, x2, x3)

    def run(self):
        raise NotImplementedError

    def stop(self):
        print("Exit handler")
        self.u.stop()