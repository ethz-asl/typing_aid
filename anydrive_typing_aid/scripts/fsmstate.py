#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String
import anydrive_msgs.msg as msg_defs
import anydrive_msgs.srv as srv_defs

import numpy as np

def set_FSM_state_client():
    rospy.init_node('set_FSM_state_client')
    rospy.loginfo("===========================================")
    selection = input("Type FSM state\n ")
    rospy.loginfo("-------------------------------------------")
    srv_name = '/anydrive/set_goal_state'
    rospy.wait_for_service(srv_name)
    try:
        set_state = rospy.ServiceProxy(srv_name, srv_defs.SetFsmGoalState)
        msg = msg_defs.FsmState()
        msg.state = np.uint8(selection)
        set_state('anydrive', msg)
    except rospy.ServiceException as e:
        print("Service call failed : %s "%e)
    rospy.loginfo("-------------------------------------------")
    rospy.loginfo("FSM state changed")

if __name__=="__main__":
    while not rospy.is_shutdown():
            set_FSM_state_client()