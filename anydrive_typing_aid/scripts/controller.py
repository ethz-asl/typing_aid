#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Header
import anydrive_msgs.msg as msg_defs


try:
    input = raw_input
except:
    pass

#Set global variables 
#TODO find a better way to do that
position = []
position.append(("up_position", 0))#angle in [rad]
position.append(("down_position", 0))
position.append(("up_limit", 0))
position.append(("down_limit", 0))
r = 6e-2 #radius of spool in [m]

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
        self.pub_target = rospy.Publisher(
            self.prefix + "/anydrive/command", msg_defs.Command, queue_size=10
        )

        def command_callback(self, data):
            self.mode = data[1]
            self.motor_position =data[2]
            self.motor_velocity = data[3]
            self.gear_position = data[4]
            self.gear_velocity = data[5]
            self.joint_position = data[6]
            self.joint_velocity = data[7]
            self.joint_torque = data[8]
            #also pid gains for the last 3 var

        #listener for inputs for same var as above but listen to the vector version 
        self.lis_target = rospy.Subscriber(
            self.prefix +"/anydrive/command", msg_defs.Commands, command_callback
        ) 

"""         def state_callback(self, data):
            self.state = data 

        #listener for motor state
        self.lis_motor_state = rospy.Subscriber(
            self.prefix +"/anydrive/state", msg_defs.Reading, state_callback
        )

        self.pub_motor_state = rospy.Publisher(
            self.prefix +"/anydrive/state", msg_defs.State, queue_size =10
        ) """

        rospa.loginfo("=================================")
        rospy.loginfo("Controller init successful.")

    def error(self, position)
        #computes the difference between the actual position and the desired one (objective)
        #returns the difference 
            if self.joint_position is not None and position is not None
            pass
            difference = r*(position - self.joint_position)
        rospy.loginfo("difference: {}".format(difference))
        return difference
        
"""     def torque(self,pos_error,vel_error, kp, kd):
        tau = kp*(pos_error)+kd*(vel_error)
        return tau """

    def move(self,velocity, mode):
        #send the movment message to the command topic
        msg = msg_defs.Command()
        msg.mode.mode = mode 
        velocity = float(velocity)
        msg.motor_velocity = velocity
        msg.gear_velocity = velocity
        msg.joint_velocity = velocity
        self.pub_target.publish(msg)
    
    def stand_still(self)
        msg = msg_defs.Command()
        msg.mode.mode = 1
        self.pub_target(msg)

if __name__ == "__main__":
    cmd = Controller()
    try:
        rospy.loginfo("=================================")
        rospy.loginfo("standing still")
        cmd.stand_still()
        rospy.timer(3)#in seconds

        #setting the desired position
        position = position.get('down_position')
        rospy.loginfo("=================================")
        rospy.loginfo("set to down_position\n getting joint_position")

        #for now defining the variables by hand
        velocity = 0.1
        mode = 9 #joint_velocity control

        while not rospy.is_shutdown():
            
            cmd.lis_target()
            error = cmd.error(position)

            #going down, faire une fonction après
            while abs(error) > 1e-3: 
                rospy.loginfo("=================================")
                rospy.loginfo("starting motion")
                cmd.move(velocity, mode)
                error = cmd.error(position)

            rospy.loginfo("=================================")
            rospy.loginfo("standing still")
            cmd.stand_still()
            rospy.timer(5)#in seconds
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