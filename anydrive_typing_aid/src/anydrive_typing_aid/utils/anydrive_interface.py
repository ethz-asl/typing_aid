import rospy
import numpy as np

from std_msgs.msg import Empty
from anydrive_msgs.msg import Reading, Command, FsmState
from anydrive_msgs.srv import SetFsmGoalState

from anydrive_typing_aid.utils.utilities import state_translator
import anydrive_typing_aid.utils.utilities as utilities


class AnydriveInterface:
    def __init__(self):
        self.prefix = "anydrive"
        self.pub_command = rospy.Publisher(
            self.prefix + "/anydrive/command", Command, queue_size=10, latch=True
        )

        srv_name = "anydrive/set_goal_state"
        rospy.wait_for_service(srv_name)
        self.srv_set_state = rospy.ServiceProxy(srv_name, SetFsmGoalState)

    def set_fsm_state(self, state):
        assert type(state) is int
        try:
            msg = FsmState()
            msg.state = np.uint8(state)
            device_name = "anydrive"
            self.srv_set_state(device_name, msg)
        except rospy.ServiceException as e:
            print("Service call failed : %s " % e)
        state_string = state_translator(state)
        rospy.loginfo("FSM state changed to {}".format(state_string))

    def get_state(self):
        msg = rospy.wait_for_message(self.prefix + "/anydrive/reading", Reading)
        return (
            msg.state.joint_position,
            msg.state.joint_velocity,
            msg.state.joint_torque,
            msg.state.current,
        )

    def callback(self, msg):
        rospy.loginfo("saved value")
        self.index = False

    def set_pos(self, param):
        rospy.Subscriber("lift_arm", Empty, self.callback)
        rate = rospy.Rate(param["rate"])
        choice = None
        # going into control op state
        self.set_fsm_state(4)
        # starting loop for calibration
        while choice < 5:
            choice = input(
                "Choose param to set  \n 1 = up_lim \n 2 = up_pos \n 3 = down_pos \n 4 = down_lim \n 5 = exit"
            )
            if choice < 5:
                self.index = True
            while self.index:
                # sending the desired torque to the drive
                msg_t = Command()
                msg_t.mode.mode = np.uint16(10)
                msg_t.joint_torque = float(0)
                self.pub_command.publish(msg_t)
                rate.sleep()
            msg = rospy.wait_for_message(self.prefix + "/anydrive/reading", Reading)
            if choice == 1:
                param["x_end_lim"] = msg.state.joint_position
            elif choice == 2:
                param["x_end"] = msg.state.joint_position
            elif choice == 3:
                param["x_0"] = msg.state.joint_position
            elif choice == 4:
                param["x_0_lim"] = msg.state.joint_position

            msg = Command()
            msg.mode.mode = 1
            self.pub_command.publish(msg)
        return param

    def move(self, mode, position=None, velocity=None, torque=None, params=None):
        msg = Command()
        msg.mode.mode = mode
        if mode in (
            utilities.MODE_ID_JOINT_POS,
            utilities.MODE_ID_JOINT_POS_VEL,
            utilities.MODE_ID_JOINT_POS_VEL_TRQ,
        ):
            assert position is not None
            msg.joint_position = position
        if mode in (
            utilities.MODE_ID_JOINT_VEL,
            utilities.MODE_ID_JOINT_POS_VEL,
            utilities.MODE_ID_JOINT_POS_VEL_TRQ,
        ):
            assert velocity is not None
            msg.joint_velocity = velocity
        if mode in (utilities.MODE_ID_JOINT_TRQ, utilities.MODE_ID_JOINT_POS_VEL_TRQ):
            assert torque is not None
            msg.joint_torque = torque
        if params is not None and "pid_p" in params:
            msg.pid_gains_p = params["pid_p"]
            msg.pid_gains_i = params["pid_i"]
            msg.pid_gains_d = params["pid_d"]
        self.pub_command.publish(msg)

    def lim_check(self, param, t_meas_, p_meas):
        if p_meas < param["x_0_lim"] or p_meas > param["x_end_lim"]:
            print("pos_stop")
            return True
        if len(t_meas_) < 5:
            return False
        if (
            np.mean(t_meas_[-5:]) > param["tau_max"]
            or np.mean(t_meas_[-5:]) < param["tau_min"]
        ):
            print("tau_stop")
            return True

    # gives the value t_next when the required value is too high or too low
    def get_lim_val(self, t_meas, position):
        if t_meas > position["tau_max"]:
            return position["tau_max"]
        if t_meas < position["tau_min"]:
            return position["tau_min"]

    def stop_drive(self):
        rospy.loginfo("Stopping drive")
        msg = Command()
        msg.mode.mode = utilities.MODE_ID_FREEZE
        self.pub_command.publish(msg)
        rospy.loginfo("Stopped drive")
