#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi

prefix = "/anydrive"
mode = None

def command_callback(data):
    global mode 
    mode = data.commanded.mode
    

if __name__=="__main__":
    rospy.init_node('data_reader', anonymous=True)
    #pub_target = rospy.Publisher(prefix +"/anydrive/command", msg_defs.Command, queue_size=10)
    lis_target = rospy.Subscriber(prefix +"/anydrive/reading", msg_defs.Reading, command_callback)#calls this function everytime it subscribes
    print(mode)
    rospy.spin()#to keep it running otherwise just executed once and then exits
    