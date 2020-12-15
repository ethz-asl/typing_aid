#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
from scipy import signal
import fsmstate as fsm
import init_mov 

global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10

# r = float(6e-2) #radius of spool in [m]

class utils(position):

    def __init__(self):
        self.prefix = "/anydrive"
        # drive_name = "anydrive"

        # rospy.init_node("controller", anonymous=True)

        # Publisher for : joint_position
        # joint_velocity
        # joint_torque
        # gear_position
        # gear_velocity
        # PID param ...
        self.pub_target = rospy.Publisher(self.prefix + "/anydrive/command", msg_defs.Command, queue_size=10)   
    
    def listener(self):
        msg = rospy.wait_for_message(self.prefix + "/anydrive/reading", msg_defs.Reading)
        self.joint_position = msg.state.joint_position
        print(self.joint_position)
        self.joint_velocity = msg.state.joint_velocity
        self.joint_torque = msg.state.joint_torque

        return self.joint_torque, self.joint_velocity, self.joint_position

    # def pid(self,mode):
    #     msg = msg_defs.Command()
    #     msg.mode.mode = np.uint16(mode) 
    #     p = input("Enter p: ")
    #     msg.pid_gains_p = float(p)
    #     i = input("Enter i: ")
    #     msg.pid_gains_i = float(i)
    #     d = input("Enter d: ")
    #     msg.pid_gains_d = float(d)
    #     self.pub_target.publish(msg)

    def error(self, mode, position, velocity, torque):
        #computes the difference between the actual position/velocity/torque and the desired one 

        if self.joint_position is not None and position is not None:
            pass
        if mode in (JOINT_POSITION,11,12):  # Tracks joint position
            #difference = r*(position - self.joint_position)
            self.difference = (position - self.joint_position)
            rospy.loginfo("position difference: {}".format(difference))
        if mode in (JOINT_VELOCITY,11,12): # Tracks joint velocity
            self.difference = (position - self.joint_velocity)
            rospy.loginfo("velocity difference: {}".format(difference))
        if mode in (JOINT_TORQUE,12): # Tracks joint torque
            self.difference = (position - self.joint_torque)
            rospy.loginfo("torque difference: {}".format(difference)) 
        return self.difference

    def move(self,mode,position,velocity, torque):
        msg = msg_defs.Command()
        msg.mode.mode = np.uint16(mode)
        if mode in (JOINT_POSITION,11,12): 
            msg.joint_position = position
        if mode in (JOINT_VELOCITY,11,12):
            velocity = float(velocity)
            msg.motor_velocity = velocity
            msg.gear_velocity = velocity
            msg.joint_velocity = velocity
        if mode in (JOINT_TORQUE,12):
            torque = float(torque)
            msg.joint_torque = torque
        self.pub_target.publish(msg)

    def lim_check(self,i,position):
        if self.joint_position < position["up_limit"] or self.joint_position > position["down_limit"] or i == 99:
            return True
        # elif self.joint_velocity > abs(position["v_max"]):
        #     return True
        # elif self.joint_torque > position["t_max"] or self.joint_torque < position["t_min"]:
        #     return True
            
    def init_pos(self,p_meas,position):
        if p_meas < position["up_position"]:
            self.move = 1
        elif p_meas > position["up_position"]:
            self.move = 0
        else:
            raise rospy.ROSInterruptException
        return self.move

    def stand_still(self,time):
        rospy.loginfo("=================================")
        rospy.loginfo("standing still")
        msg = msg_defs.Command()
        msg.mode.mode = 1
        self.pub_target.publish(msg)
        rospy.sleep(time)#in seconds

    def stop(self):
        rospy.loginfo("Stopping drive")
        msg = msg_defs.Command()
        msg.mode.mode = 1
        self.pub_target.publish(msg)
        #goes to configure
        fsm.FSM_state().set_FSM_state(3)

    #https://en.wikipedia.org/wiki/Logistic_function
    def logistic_fct(self,x,midpoint,max_val, steepness):
        self.fct = max_val/(1+np.exp(-steepness*(x-midpoint)))
        return self.fct

