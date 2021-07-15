#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String
import anydrive_msgs.msg as msg_defs
import anydrive_msgs.srv as srv_defs

import numpy as np


class FSM_state:
    def __init__(self):
        srv_name = "/anydrive/set_goal_state"
        rospy.wait_for_service(srv_name)
        self.set_state = rospy.ServiceProxy(srv_name, srv_defs.SetFsmGoalState)

    def set_FSM_state(self, selection):
        try:
            msg = msg_defs.FsmState()
            msg.state = np.uint8(selection)
            self.set_state("anydrive", msg)
        except rospy.ServiceException as e:
            print("Service call failed : %s " % e)
        rospy.loginfo("-------------------------------------------")
        rospy.loginfo("FSM state changed")