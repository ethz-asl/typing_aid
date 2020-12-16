#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
from scipy import signal
import fsmstate as fsm
from itertools import cycle 
# import plot as plt

global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10

# TBD
position = {
    "up_position": 4,
    "down_position": 0,
    "up_limit": 5,
    "down_limit": -2.5,
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

        return self.joint_torque,self.joint_velocity, self.joint_position

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

    def lim_check(self,i):
        if self.joint_position < position["up_limit"] or self.joint_position > position["down_limit"] or i == 199:
            return True
        # elif self.joint_velocity > abs(position["v_max"]):
        #     return True
        # elif self.joint_torque > position["t_max"] or self.joint_torque < position["t_min"]:
        #     return True
            
    def init_pos(self,p_meas):
        if p_meas < position["up_position"]:
            move = True
        elif p_meas > position["up_position"]:
            move = False
        else:
            raise rospy.ROSInterruptException
        return move

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
        i = True
        t_meas, v_meas, p_meas = cmd.listener()
        mode = 10 
        # mode = input("Choose control mode : \n pos = 8 \n vel = 9 \n torque = 10 \n pos_vel = 11 \n pos_vel_t = 12 \n") 
        # if mode in (JOINT_VELOCITY,11,12):
        #     v_des = input("desired velocity")
        # if mode in (JOINT_TORQUE,12):
        #     t_des = input("desired torque")
        
        #setting FSM_state
        #goes into ControlOp
        fsm_state = fsm.FSM_state()
        fsm_state.set_FSM_state(4)
        #cmd.pid(mode)
        init = True

        while not rospy.is_shutdown():
            
            i = not i
            t_meas, v_meas, p_meas = cmd.listener() 
            rospy.loginfo("measured torque: {}".format(t_meas))
            rospy.loginfo("measured pos: {}".format(p_meas))

            if init:
                i = cmd.init_pos(p_meas)
                init = False
            t=np.linspace(-5,5,200)
            if  i: #go up
                # p_des = input("Enter up position: ")
                p_des = position["up_position"]
                t_des = t_meas + np.sign(t_meas)*0.3
                path = cmd.logistic_fct(t,t_des/1.5,t_des + np.sign(t_meas)*0.1,0.75)
                rospy.loginfo("going up")

            elif not i: #go down
                # p_des = input("Enter down position: ")
                p_des = position["down_position"]
                t_des = t_meas - np.sign(t_meas)*0.1
                # t_des = t_meas
                path = cmd.logistic_fct(-t,t_des/1.5,t_des- np.sign(t_meas)*0.1,0.75)
                rospy.loginfo("going down")
            else: 
                raise rospy.ROSInterruptException

            rospy.loginfo("applied torque: {}".format(t_des))
            tst = input("type a number to continue ")
            rate = rospy.Rate(200) # 200hz
            error = []
            l = 0
            error.append(cmd.error(JOINT_POSITION, p_des, v_des, t_des)) 
            rospy.loginfo("starting motion")    

            while abs(error[l])>=0.1:
                t_next = path[l]
                cmd.move(mode,p_des, v_des, t_next)
                t_meas, v_meas, p_meas = cmd.listener()
                error.append(cmd.error(JOINT_POSITION, p_des, v_des, t_des))
                rospy.loginfo("applied torque: {}".format(t_next))
                # if cmd.lim_check(l):
                #     raise rospy.ROSInterruptException
                l+=1
                rate.sleep()
            # needed to freeze motor
            cmd.stand_still(1)
            # plt().plot(error, False)
    except rospy.ROSInterruptException:
        cmd.stop()
