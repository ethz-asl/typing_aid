#!/usr/bin/env python
# coding=utf-8

import rospy
import numpy as np
import utils
import cte_mov
import pid
import pos_control as p_cont

global JOINT_POSITION,JOINT_VELOCITY,JOINT_TORQUE
JOINT_POSITION = 8
JOINT_VELOCITY = 9
JOINT_TORQUE =10

class save:
    def method_selection(self):
        controller = input("Choose the controller method  \n 1 = position controller \n 2 = torque controller \n 3 = pid controller \n")
        # number of cycles
        number = input("Choose the number of cycles to execute  \n ")
        if controller == 1:
            self.out = p_cont.pos_mov().run(number)
        elif controller == 2:
            cte_mov.cte_mov().run(number)
        elif controller == 3:
            self.out = pid.pid().run(number)
        # store the data
        # self.data.append([controller, number, 999])
        # self.save_to_csv()

    # def add_data_col(self, new_data, ax):
    #     # add the value in a new col
    #     self.data = []
    #     if not self.data:
    #         self.data = new_data
    #     else:
    #         if ax == 1:
    #             self.data = np.concatenate((self.data,new_data), axis = ax)
    #         # add data in a new row
    #         else:
    #             self.data = np.concatenate((self.data,new_data), axis = ax)
    #     return self.data
    
    # def save_to_csv(self):
    #     df = np.array(self.out)
    #     print(df)
    #     print(df.shape)
    #     np.savetxt('data.csv', df, delimiter=',')