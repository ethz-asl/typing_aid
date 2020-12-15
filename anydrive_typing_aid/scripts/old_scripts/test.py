#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import anydrive_msgs.srv as srv_defs



def set_state_client(selection):
    srv_name = '/anydrive/send_controlword'
    rospy.wait_for_service(srv_name)
    try:
        set_state = rospy.ServiceProxy(srv_name, srv_defs.SendControlword)
        path = set_state('anydrive',np.uint16(selection))
    except rospy.ServiceException as e:
        print("Service call failed : %s "%e)

if __name__=="__main__":
    while not rospy.is_shutdown():
            rospy.init_node('service_client')
            rospy.loginfo("===========================================")
            selection = input(
                "Type state\n "
            )
            rospy.loginfo("-------------------------------------------")
            set_state_client(selection)


