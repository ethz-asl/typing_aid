#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
import fsmstate as fsm
import pid


global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10

class utils:

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
        # print(self.joint_position)
        self.joint_velocity = msg.state.joint_velocity
        self.joint_torque = msg.state.joint_torque
        return self.joint_torque, self.joint_velocity, self.joint_position
# mettre aussi le moteur dans une bonne config pour le tourner comme on veut (torque control and cte 0 torque applied)
    def set_pos(self, param): 
        rate = param["rate"]
        choice = None 
        # going into control op state 
        fsm.FSM_state().set_FSM_state(4)
        #starting loop for calibration
        while choice is not 5:
            choice = input("Move the drive to the correct position and type the desired pos  \n 1 = up_lim \n 2 = up_pos \n 3 = down_pos \n 4 = down_lim \n 5 = exit")
            msg = rospy.wait_for_message(self.prefix + "/anydrive/reading", msg_defs.Reading)
            while not choice:
                # sending the desired torque to the drive 
                rospy.loginfo("sending 0 torque to calibrate")
                msg_t = msg_defs.Command()
                msg_t.mode.mode = np.uint16(10)
                msg_t.joint_torque = float(0)
                self.pub_target.publish(msg_t)
                rate.sleep()
            if choice == 1:
                param['x_end_lim'] = msg.state.joint_position
            elif choice == 2:
                param['x_end'] = msg.state.joint_position
            elif choice == 3:
                param['x_0'] = msg.state.joint_position
            elif choice == 4:
                param['x_0_lim'] = msg.state.joint_position
            else:
                choice == 5
        #freezing the drive
        msg = msg_defs.Command()
        msg.mode.mode = 1
        self.pub_target.publish(msg)
        #going into configure state
        fsm.FSM_state().set_FSM_state(3)
        return param
            

    # to change pid gains from motor
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

    def set_PID(self):
        p = input("p_gain")
        i = input("i_gain")
        d = input("d_gain")
        # initialization of p_error
        p_error = input("p_error")
        return p,i,d,p_error

    # def error(self, mode, position, velocity, torque):
    #     #computes the difference between the actual position/velocity/torque and the desired one 

    #     if self.joint_position is not None and position is not None:
    #         pass
    #     if mode in (JOINT_POSITION,11,12):  # Tracks joint position
    #         #difference = r*(position - self.joint_position)
    #         difference = (position - self.joint_position)
    #         rospy.loginfo("position difference: {}".format(difference))
    #     if mode in (JOINT_VELOCITY,11,12): # Tracks joint velocity
    #         difference = (position - self.joint_velocity)
    #         rospy.loginfo("velocity difference: {}".format(difference))
    #     if mode in (JOINT_TORQUE,12): # Tracks joint torque
    #         difference = (position - self.joint_torque)
    #         rospy.loginfo("torque difference: {}".format(difference)) 
    #     return difference

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

    def lim_check(self,position):
        # if self.joint_position < position["up_limit"] or self.joint_position > position["down_limit"]:
            # return True
        # elif self.joint_velocity > abs(position["v_max"]):
        #     return True
        if self.joint_torque > position["t_max"] or self.joint_torque < position["t_min"]:
            return True
    # gives the value t_next when the required value is too high or too low
    def get_lim_val(self, t_meas, position):
        if self.joint_torque > position["t_max"]:
            return position["t_max"]
        if self.joint_torque < position["t_min"]:
            return position["t_min"]

    # def init_pos(self,p_meas,position):
    #     if p_meas < position["up_position"]:
    #         self.go = 1
    #     elif p_meas > position["up_position"]:
    #         self.go = 0
    #     else:
    #         raise rospy.ROSInterruptException
    #     return self.go

    # def stand_still(self,time):
    #     rospy.loginfo("=================================")
    #     rospy.loginfo("standing still")
    #     msg = msg_defs.Command()
    #     msg.mode.mode = 1
    #     self.pub_target.publish(msg)
    #     rospy.sleep(time)#in seconds

    def stop(self):
        rospy.loginfo("Stopping drive")
        msg = msg_defs.Command()
        # freezing the drive
        msg.mode.mode = 1
        self.pub_target.publish(msg)
        #goes to configure
        fsm.FSM_state().set_FSM_state(3)

    #https://en.wikipedia.org/wiki/Logistic_function
    def logistic_fct(self,x,midpoint,max_val, steepness):
        self.fct = max_val/(1+np.exp(-steepness*(x-midpoint)))
        return self.fct

    # used to compute the point for the quadratic fct 
    # set const to zero to begin
    def quadratic_fct(self,t0, t_end, tau_0, tau_end, sampling_time):
        t_des = float(t_end-t0)/2.0 +t0
        A = np.array([[2*t0, 1, 0, 0, 0, 0],
            [0, 0, 0, 2*t_end, 1, 0],
            [t_des**2, t_des, 1, -t_des**2, -t_des, -1],
            [2*t_des, 1, 0, -2*t_des, -1, 0],
            [t0**2, t0, 1, 0, 0, 0],
            [0, 0, 0, t_end**2, t_end, 1]])
        b = np.array([0, 0, 0, 0, tau_0, tau_end])
        b = b.transpose()
        c = np.linalg.inv(A)
        c = c.dot(b)

        duration = t_end - t0
        num_samples = round(float(duration) / float(sampling_time))
        half_num_samples = int(num_samples/2.0)

        x1 = np.linspace(t0, t_des, half_num_samples, endpoint=True)
        y1 = c[0]*x1**2+c[1]*x1+c[2]

        x2 = np.linspace(t_des, t_end, half_num_samples, endpoint=True)
        y2 = c[3]*x2**2+c[4]*x2+c[5]

        x = np.concatenate((x1,x2))
        y = np.concatenate((y1,y2))
        return x,y

    # to generate straight line. Duration in s.
    def const(self,tau, t0, t_end, sampling_time):
        duration = t_end - t0
        num_samples = round(float(duration) / float(sampling_time))
        x = np.linspace(t0, t_end, num_samples, endpoint=True)
        y = tau * np.ones(int(num_samples))
        return x,y
    
    def afine(self, a, tau_0, t0, sampling_time):
        duration = 50
        num_samples = round(float(duration) / float(sampling_time))
        x = np.linspace(t0, t0 + 50, num_samples, endpoint=True)
        y = a * x + tau_0
        return x,y 

    # put the pieces together
    def torque_profile(self,y1,y2,y3,x1,x2,x3):
        # assert y1[-1] == y2[0] and y2[-1] == y3[0]
        # assert x1[-1] == x2[0] and x2[-1] == x3[0]
        x = np.concatenate((x1,x2,x3))
        y = np.concatenate((y1,y2,y3))
        return x,y 

    def store(self, t_meas, v_meas, p_meas,t_meas_,v_meas_,p_meas_):
        t_meas_.append(t_meas)
        v_meas_.append(v_meas)
        p_meas_.append(p_meas)
        return t_meas_,v_meas_,p_meas_
    
    def store_one(self, var, array):
        array.append(var)
        return array

    def plot(self, x, y , title):
        plt.figure()
        plt.plot(x,y)
        plt.savefig(self.get_time() + title)
    
    def check_sign(self, t_meas,t_next):
        if (t_meas-t_next) > 0:
            return -1
        else:
            return 1

    def get_time(self):
        date = datetime.now()
        name = date.strftime("%d/%m/%Y, %H:%M:%S")
        return name
