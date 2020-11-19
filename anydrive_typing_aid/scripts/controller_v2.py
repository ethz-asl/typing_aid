#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs
from math import pi

#Set global variables 
#TODO find a better way to do that
#angle in rad
position = {
    "up_position": float(2*pi),
    "down_position": float(0),
    "up_limit": float(0),
    "down_limit": float(0),
}

r = float(6e-2) #radius of spool in [m]

class Controller:

    def __init__(self):
        self.prefix = "/anydrive"
        # drive_name = "anydrive"

        rospy.init_node("controller", anonymous=True)

        self.mode = None
        self.motor_position =None
        self.motor_velocity = None
        self.gear_position = None
        self.gear_velocity =None
        self.joint_position = None
        self.joint_velocity = None
        self.joint_torque = None
        self.pid_gains_p = None
        self.pid_gains_i = None
        self.pid_gains_d = None
        self.state = None

        # Publisher for : joint_position
        # joint_velocity
        # joint_torque
        # gear_position
        # gear_velocity
        # PID param ...
        self.pub_target = rospy.Publisher(self.prefix + "/anydrive/command", msg_defs.Command, queue_size=10)

        def command_callback(data):
            self.mode = data.commanded.mode
            self.motor_position = data.commanded.motor_position
            self.motor_velocity = data.commanded.motor_velocity
            self.gear_position = data.commanded.gear_position    
            self.gear_velocity = data.commanded.gear_velocity
            self.joint_position = data.commanded.joint_position
            self.joint_velocity = data.commanded.joint_velocity
            self.joint_torque = data.commanded.joint_torque
            self.pid_gains_p = data.commanded.pid_gains_p
            self.pid_gains_i = data.commanded.pid_gains_i
            self.pid_gains_d = data.commanded.pid_gains_d

            self.state = data.state

        self.lis_target = rospy.Subscriber(self.prefix +"/anydrive/reading", msg_defs.Reading, command_callback)

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
            difference = r*(position - self.joint_position)
        elif mode == 9: # Tracks joint velocity
            difference = r*(position - self.joint_velocity)
        elif mode == 10: # Tracks joint torque
            difference = r*(position - self.joint_torque)
        rospy.loginfo("difference: {}".format(difference))
        return difference
        
    def move(self,velocity,mode):
        #send the movment message to the command topic
        msg = msg_defs.Command()
        msg.mode.mode = np.uint16(mode) 
        velocity = float(velocity)
        msg.motor_velocity = velocity
        msg.gear_velocity = velocity
        msg.joint_velocity = velocity
        self.pub_target.publish(msg)
    
    def stand_still(self,time):
        rospy.loginfo("=================================")
        rospy.loginfo("standing still")
        msg = msg_defs.Command()
        msg.mode.mode = 1
        self.pub_target.publish(msg)
        rospy.sleep(time)#in seconds

if __name__ == "__main__":
    cmd = Controller()
    rospy.loginfo("=================================")
    rospy.loginfo("Controller init successful.")
    try:
        #setting the desired position
        position = position.get('down_position')
        rospy.loginfo("=================================")
        rospy.loginfo("set to down_position\n getting joint_position")

        #for now defining the variables by hand
        velocity = 0.1
        mode = 8 #joint_position control

        #cmd.pid(mode)

        while not rospy.is_shutdown():
            cmd.stand_still(3)

            error = cmd.error(position,mode)

            #going down, faire une fonction après
            while abs(error) > 1e-3: 
                cmd.lis_target
                rospy.loginfo("=================================")
                rospy.loginfo("starting motion")
                cmd.move(velocity, mode)
                error = cmd.error(position)

            rospy.loginfo("=================================")
            rospy.loginfo("standing still")
            cmd.stand_still(3)
            #switching to up position
            position = position.get('up_position')

            #going_up faire une fonction pour ça
            while abs(error) > 1e-3:
                rospy.loginfo("=================================")
                rospy.loginfo("starting motion")
                cmd.move(velocity, mode)
                error = cmd.error(position) 

    except rospy.ROSInterruptException:
        cmd.stand_still()


#TODO Define a security limit for up and down positions, def a max velocity, acceleration
#TODO Think of a smooth transition: peut etre en prenant les valeurs du gear et du joint, en faisant la diff
# et en utilisant la raideur du ressort 