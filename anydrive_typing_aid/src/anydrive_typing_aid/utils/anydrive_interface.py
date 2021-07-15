import rospy
import numpy as np

from anydrive_msgs.msg import Reading, Command, FsmState
from anydrive_msgs.srv import SetFsmGoalState

from anydrive_typing_aid.utils.utilities import state_translator


class AnydriveInterface:
    def __init__(self):
        self.prefix = "/anydrive"
        self.pub_command = rospy.Publisher(
            self.prefix + "/anydrive/command", Command, queue_size=10
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

    def listener(self):
        msg = rospy.wait_for_message(self.prefix + "/anydrive/reading", Reading)
        self.joint_position = msg.state.joint_position
        self.joint_velocity = msg.state.joint_velocity
        self.joint_torque = msg.state.joint_torque
        self.motor_current = msg.state.current
        return (
            self.joint_torque,
            self.joint_velocity,
            self.joint_position,
            self.motor_current,
        )

    def callback(self, msg):
        rospy.loginfo("saved value")
        self.index = False

    def set_pos(self, param):
        rospy.Subscriber("lift_arm", Empty, self.callback)
        rate = rospy.Rate(param["rate"])
        choice = None
        # going into control op state
        fsm.FSM_state().set_FSM_state(4)
        # starting loop for calibration
        while choice < 5:
            choice = input(
                "Choose param to set  \n 1 = up_lim \n 2 = up_pos \n 3 = down_pos \n 4 = down_lim \n 5 = exit"
            )
            if choice < 5:
                self.index = True
            while self.index:
                # sending the desired torque to the drive
                msg_t = msg_defs.Command()
                msg_t.mode.mode = np.uint16(10)
                msg_t.joint_torque = float(0)
                self.pub_command.publish(msg_t)
                rate.sleep()
            msg = rospy.wait_for_message(
                self.prefix + "/anydrive/reading", msg_defs.Reading
            )
            if choice == 1:
                param["x_end_lim"] = msg.state.joint_position
            elif choice == 2:
                param["x_end"] = msg.state.joint_position
            elif choice == 3:
                param["x_0"] = msg.state.joint_position
            elif choice == 4:
                param["x_0_lim"] = msg.state.joint_position

            msg = msg_defs.Command()
            msg.mode.mode = 1
            self.pub_command.publish(msg)
        return param

    # to change pid gains of motor
    def pid(self, mode, param):
        msg = msg_defs.Command()
        msg.mode.mode = np.uint16(mode)
        p = param["p"]
        msg.pid_gains_p = float(p)
        i = param["i"]
        msg.pid_gains_i = float(i)
        d = param["d"]
        msg.pid_gains_d = float(d)
        self.pub_command.publish(msg)

    def set_PID(self, param):
        param["p_gain"] = input("p_gain")
        param["i_gain"] = input("i_gain")
        param["d_gain"] = input("d_gain")
        # initialization of p_error
        param["p_error"] = input("p_error")
        return param

    def move(self, mode, position, velocity, torque):
        msg = msg_defs.Command()
        msg.mode.mode = np.uint16(mode)
        if mode in (JOINT_POSITION, 11, 12):
            msg.joint_position = position
        if mode in (JOINT_VELOCITY, 11, 12):
            velocity = float(velocity)
            msg.motor_velocity = velocity
            msg.gear_velocity = velocity
            msg.joint_velocity = velocity
        if mode in (JOINT_TORQUE, 12):
            torque = float(torque)
            msg.joint_torque = torque
        self.pub_command.publish(msg)

    def lim_check(self, param, t_meas_, p_meas):
        if p_meas < param["x_0_lim"] or p_meas > param["x_end_lim"]:
            print("pos_stop")
            return True
        if len(t_meas_) < 5:
            return False
        if (
            mean(t_meas_[-5:]) > param["tau_max"]
            or mean(t_meas_[-5:]) < param["tau_min"]
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
        msg = msg_defs.Command()
        # freezing the drive
        msg.mode.mode = 1
        self.pub_command.publish(msg)
        rospy.loginfo("Stopped drive")

