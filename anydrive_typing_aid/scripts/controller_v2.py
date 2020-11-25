#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi
import numpy as np

#Set global variables 
#TODO find a better way to do that
#angle in rad
position = {
    "up_position": 2,
    "down_position": 0,
    "up_limit": 0,
    "down_limit": 0,
}

r = float(6e-2) #radius of spool in [m]

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

    def pid(self,mode):
        msg = msg_defs.Command()
        msg.mode.mode = np.uint16(mode) 
        p = input("Enter p: ")
        msg.pid_gains_p = float(p)
        i = input("Enter i: ")
        msg.pid_gains_i = float(i)
        d = input("Enter d: ")
        msg.pid_gains_d = float(d)
        self.pub_target.publish(msg)


    def error(self, position, mode):
        #computes the difference between the actual position and the desired one (objective)
        #returns the difference 
        if self.joint_position is not None and position is not None:
            pass
        if mode == 8:  # Tracks joint position
            #difference = r*(position - self.joint_position)
            difference = (position - self.joint_position)
        elif mode == 9: # Tracks joint velocity
            difference = r*(position - self.joint_velocity)
        elif mode == 10: # Tracks joint torque
            difference = r*(position - self.joint_torque)
        rospy.loginfo("difference: {}".format(difference))
        return difference
        
    def move_vel(self,velocity,mode):
        #send the movment message to the command topic
        msg = msg_defs.Command()
        msg.mode.mode = np.uint16(mode) 
        velocity = float(velocity)
        msg.motor_velocity = velocity
        msg.gear_velocity = velocity
        msg.joint_velocity = velocity
        self.pub_target.publish(msg)

    def move_pos(self,mode,position):
        #send the movment message to the command topic
        msg = msg_defs.Command()
        msg.mode.mode = np.uint16(mode) 
        msg.joint_position = position
        self.pub_target.publish(msg)
    
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

if __name__ == "__main__":
    cmd = Controller()
    rospy.loginfo("=================================")
    rospy.loginfo("Controller init successful.")
    try:
        #setting the desired position

        rospy.loginfo("=================================")
        rospy.loginfo("set to down_position\n getting joint_position")

        #for now defining the variables by hand
        velocity = 0.1
        mode = 8 #joint_position control

        #cmd.pid(mode)

        while not rospy.is_shutdown():

            position = input("Enter down position: ")
            cmd.listener()
            error = cmd.error(position,mode)

            #going down, faire une fonction après
            while abs(error) > 0.5: 
                rospy.loginfo("=================================")
                rospy.loginfo("starting motion")
                cmd.move_pos(mode,position)
                cmd.listener()
                error = cmd.error(position, mode)

            rospy.loginfo("=================================")
            rospy.loginfo("standing still")
            position = input("up pos ")
            cmd.listener()
            error = cmd.error(position,mode)
            #going_up faire une fonction pour ça
            while abs(error) >  0.5:
                rospy.loginfo("=================================")
                rospy.loginfo("starting motion")
                cmd.move_pos(mode,position)
                cmd.listener()
                error = cmd.error(position,mode) 
            value = input("fin ")
            cmd.stop()

    except rospy.ROSInterruptException:
        cmd.stop()


#TODO Define a security limit for up and down positions, def a max velocity, acceleration
#TODO Think of a smooth transition: peut etre en prenant les valeurs du gear et du joint, en faisant la diff
# et en utilisant la raideur du ressort 