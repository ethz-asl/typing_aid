#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi

prefix = "/anydrive"

def listener():
    rospy.init_node('extract_val', anonymous=True)
    msg = rospy.wait_for_message(prefix + "/anydrive/reading", msg_defs.Reading)
    joint_position = msg.state.joint_position
    joint_velocity = msg.state.joint_velocity
    joint_torque = msg.state.joint_torque
    return joint_position, joint_velocity, joint_torque

if __name__=="__main__":
    p,v,t = listener()
    print(p)
    print(v)
    print(t)
    