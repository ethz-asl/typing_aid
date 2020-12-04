#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
from scipy import signal
import fsmstate as fsm

global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10

# TBD
position = {
    "up_position": 0,
    "down_position": 1,
    "up_limit": -1,
    "down_limit": 5,
    "v_max": 5,
    "t_min":0.3,
    "t_max":1.5
}

# r = float(6e-2) #radius of spool in [m]

class Controller:

    def __init__(self):
        self.prefix = "/anydrive"
        # drive_name = "anydrive"

        rospy.init_node("controller", anonymous=True)

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
            difference = (position - self.joint_position)
            rospy.loginfo("position difference: {}".format(difference))
        if mode in (JOINT_VELOCITY,11,12): # Tracks joint velocity
            difference = (position - self.joint_velocity)
            rospy.loginfo("velocity difference: {}".format(difference))
        if mode in (JOINT_TORQUE,12): # Tracks joint torque
            difference = (position - self.joint_torque)
            rospy.loginfo("torque difference: {}".format(difference)) 
        return difference

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

    def lim_check(self):
        if self.joint_position > position["up_limit"] or self.joint_position < position["down_limit"]:
            return True
        elif self.joint_velocity > abs(position["v_max"]):
            return True
        elif self.joint_torque > position["t_max"] or self.joint_torque < position["t_min"]:
            return True
            
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
        return max_val/(1+np.exp(-steepness*(x-midpoint)))

if __name__ == "__main__":
    cmd = Controller()
    rospy.loginfo("=================================")
    rospy.loginfo("Controller init successful.")
    try:
        v_des = 0
        t_des = 0
        mode = input("Choose control mode : \n pos = 8 \n vel = 9 \n torque = 10 \n pos_vel = 11 \n pos_vel_t = 12 \n") 
        if mode in (JOINT_VELOCITY,11,12):
            v_des = input("desired velocity")
        if mode in (JOINT_TORQUE,12):
            t_des = input("desired torque") 
        
        #setting FSM_state
        #goes into ControlOp
        fsm_state = fsm.FSM_state()
        fsm_state.set_FSM_state(4)
    

        #cmd.pid(mode)

        while not rospy.is_shutdown():

            p_des = input("Enter down position: ")
            t=np.linspace(0.1,t_des,1e3)
            i = 0

            # p_des = position["down_position"]
            cmd.listener()
            error = cmd.error(JOINT_POSITION, p_des, v_des, t_des)
            rate = rospy.Rate(100) # 10hz
            while abs(error)>=0.1:
                t_next = cmd.logistic_fct(t[i],t_des/2,t_des,1.5)
                #going down, faire une fonction après
                rospy.loginfo("=================================")
                rospy.loginfo("starting motion")
                cmd.move(mode,p_des, v_des, t_next)
                cmd.listener()
                error = cmd.error(JOINT_POSITION, p_des, v_des, t_des)
                # if cmd.lim_check():
                #     cmd.stop()
                #     break
                i+=1
                rate.sleep()
            cmd.stand_still(3)

    except rospy.ROSInterruptException:
        cmd.stop()


#TODO Define security limits for up and down positions, def a max velocity, acceleration
#TODO Think of a smooth transition: peut etre en prenant les valeurs du gear et du joint, en faisant la diff
# et en utilisant la raideur du ressort 