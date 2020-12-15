#!/usr/bin/env python
# coding=utf-8

#idea is to create a pid with position as input and torque as output for the going down movement.

import math 
import numpy as np
import time

class pid:
    def __init__(self, p_gain, i_gain, d_gain):
        self._p_error, self._i_error, self._d_error = None, None, None
        self._p_gain, self._i_gain, self._d_gain = None, None, None
        self._last_time, self._p_error_last = None

        self.set_gains(p_gain, i_gain, d_gain)
    
    def set_gains(self, p_gain, i_gain, d_gain):
        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain
    
    def update(self, p_error):
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

        if self._p_error_last is None:
            self._p_error_last = 0
        else:
            self._p_error_last = self._p_error
        self._d_error = (self._p_error-self._p_error_last)/dt

        d = self._d_gain*self._d_error

        self.cmd = p+i+d
        return self.cmd     


#pour sortir un couple il faut la relation position couple 
