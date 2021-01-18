#!/usr/bin/env python
# coding=utf-8

#idea is to create a pid with position as input and torque as output for the going down movement.

import math 
import numpy as np
import time

import utils
import saver as s

global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10

class pid:
    def __init__(self, p_gain, i_gain, d_gain):
        self._p_error, self._i_error, self._d_error = None, None, None
        self._p_gain, self._i_gain, self._d_gain = None, None, None
        self._last_time, self._p_error_last = None, None 
        self.u = utils.utils()
        self.set_gains(p_gain, i_gain, d_gain)
        self.t_meas_, self.v_meas_, self.p_meas_, self.data = [],[],[],[]
        self.p_,self.i_,self.d_ = [],[],[]
    
    def set_gains(self, p_gain, i_gain, d_gain):
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain

    def set_param(self):
        params = {
            "t0" : 0,
            "t_end" : 10,
            "rate" : rospy.Rate(2), # in hz
            "num" : 200,
            "v_0" : 0.3,
            "v_end" : 0.7,
            "transition" : 3,
            "v_min" : -0.3,
            "v_max" : 0.5,
            "t_min": 0.25,
            "t_max": 0.75,
            "tol": 0.01

        }
        return params

# transition time is the time needed to go from low torque to high torque
# tau_0 is low torque value and tau_end is high torque value
    def compute_traj(self):
        params = self.set_param()
        # way up : 
        x1,y1 = self.u.quadratic_fct(params["t0"], params["t0"]+params["transition"], params["v_0"], params["v_end"],params["num"])
        x2,y2 = self.u.const(params["v_end"], params["t0"]+params["transition"] , params["t_end"]-params["transition"])
        x3,y3 = self.u.quadratic_fct(params["t_end"]-params["transition"],params["t_end"], params["v_end"], params["v_0"],params["num"])
        # put everything together
        return self.u.torque_profile(y1,y2,y3,x1,x2,x3)
    
    def update(self, p_error, wind_val = 0):
        t = time.time()
        if self._last_time is None:
            self._last_time = t
        dt = t-self._last_time

        if dt == 0 or math.isnan(dt) or math.isinf(dt):
            return 0

        self._p_error = p_error
        p = self._p_gain * self._p_error

        self._i_error += dt * self._p_error 
        i = self._i_gain*self._i_error
        
        i = self.antiWindup(i, wind_val)

        if self._p_error_last is None:
            self._p_error_last = 0
        else:
            self._p_error_last = self._p_error
        self._d_error = (self._p_error-self._p_error_last)/dt

        d = self._d_gain*self._d_error

        self.p_,self.i_,self.d_ = self.u.store(p,i,d,self.p_,self.i_,self.d_)

        self.cmd = p+i+d
        return self.cmd    

    # t_meas is the measured curent torque, t_next is the desired torque sent to the drive, tol is the maximum delta we allow
    # defined in params
    def filt(self,t_meas,t_next,params):
        # if true, values are out of bounds
        if self.u.lim_check(params):
            rospy.loginfo("value out of bounds")
            t_next = get_lim_val(t_meas, params)
        if abs(t_meas-t_next) >= params["tol"]:
            cte = self.u.check_sign(t_meas,t_next)
            rospy.loginfo("step to big, adapting")
            t_next = t_meas + params["tol"]
        return t_next

    def antiWindup(self, i, wind_val):
        if i >= wind_val:
            i = wind_val
        elif i <= -wind_val:
            i = -wind_val
        return i


# n is the number of time the trajectory path (of velocity) is taken
    def move(self,n):
        # set the PID gains and pid_error to 0 and does one update
        self.u.set_PID()
        rospy.loginfo("getting parameters")
        params = self.set_param()
        rate = params["rate"]
        rospy.loginfo("computing trajectory")
        x,p_des = self.compute_traj()
        # define some velocity input profile 
        l = 0
        while n>=1: 
            while l <= (len(y)-1):
                # p_des is unsused here, just defined to make it worked
                p_des = 0
                # x, v_des = put the function here for the position
                t_meas, v_meas, p_meas = self.u.listener()
                p_error = p_meas - p_des[l] 
                t_next = self.update(p_error)
                t_next = filt(t_meas,t_next,params)
                self.u.move(JOINT_TORQUE,p_des, self.v_des, t_next)
                rospy.loginfo("applied torque: {}".format(t_next))
                if self.u.lim_check(params):
                    raise rospy.ROSInterruptException
                # store the values
                # l à changer parce que de 0 à 10 pour l'instant
                self.t_meas_, self.v_meas_, self.p_meas_ = self.u.store(t_meas, v_meas, p_error,self.t_meas_, self.v_meas_, self.p_meas_)
                l+=1
                rate.sleep()
            n = n-1 
            l = 0

    # plotting the desired path
        x = np.arange(0, len(self.t_meas_), 1)
        self.u.plot(x, p_des , "desired_traj.png")
        self.u.plot(x, self.t_meas_ , "torque.png")
        self.u.plot(x, self.p_error_ , "velocity_error.png")

    # saving the data
        self.data = s.save().add_data_col([self.t_meas_,self.v_meas_,self.p_meas_], ax = 0)
        # saves pid gains in another row (=below)
        self.data = s.save().add_data_col([self.p_,self.i_,self.d_], ax = 0)

    def run(self,n):
        rospy.loginfo("starting movement")
        self.move(n)
        self.u.stop()