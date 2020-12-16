#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import fsmstate as fsm
import utils 

global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10


class init_mov:

    def __init__(self):
        self.v_des, self.t_des, self.i,self.l = 0,0,0,0

    def run(self,init):

            u= utils()
            position = u.set_pos()

            while not rospy.is_shutdown():

                t_meas, p_meas = u.listener() 
                rospy.loginfo("measured torque: {}".format(t_meas))
                rospy.loginfo("measured pos: {}".format(p_meas))

                self.i = u.init_pos(p_meas,position)

                t=np.linspace(-5,5,100)

                if self.i == 0:#go up
                    # p_des = input("Enter up position: ")
                    p_des = position["up_position"]
                    self.t_des = t_meas + np.sign(t_meas)*0.3
                    path = u.logistic_fct(t,self.t_des/1.5,self.t_des + np.sign(t_meas)*0.1,0.75)
                    rospy.loginfo("going up")

                elif self.i == 1: #go down
                    # p_des = input("Enter down position: ")
                    p_des = position["down_position"]
                    # self.t_des = t_meas - np.sign(t_meas)*0.1
                    self.t_des = t_meas
                    path = u.logistic_fct(t,self.t_des/1.5,self.t_des- np.sign(t_meas)*0.1,0.75)
                    rospy.loginfo("going down")
                else: 
                    raise rospy.ROSInterruptException

                rospy.loginfo("applied torque: {}".format(self.t_des))

                tst=input("type a number to continue ")

                error = u.error(JOINT_POSITION, p_des, self.v_des, self.t_des)
                rate = rospy.Rate(200) # 200hz

                rospy.loginfo("starting motion")    

                while abs(error)>=0.1:
                    t_next = path[self.l]
                    u.move(mode,p_des, self.v_des, t_next)
                    t_meas = u.listener()
                    error = u.error(JOINT_POSITION, p_des, self.v_des, self.t_des)
                    rospy.loginfo("applied torque: {}".format(t_next))
                    if u.lim_check(self.l,position):
                        raise rospy.ROSInterruptException
                    self.l+=1
                    rate.sleep()
                rospy.loginfo("up position reached \n end of initialisation") 
                if init:
                    break
