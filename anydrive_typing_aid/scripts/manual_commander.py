#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import anydrive_msgs.msg as msg_defs

try:
    input = raw_input
except:
    pass


class ManualCommander:
    def __init__(self):
        self.prefix = "/anydrive"
        # drive_name = "anydrive"

        rospy.init_node("manual_commander", anonymous=True)

        # Publishers
        self.cmd_pub = rospy.Publisher(
            self.prefix + "/anydrive/command", msg_defs.Command, queue_size=10
        )

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("===========================================")
            selection = input(
                "Select from:\n (a) Abort\n (b) Print state\n (c) Command position\n (d) Command velocity\n> "
            )
            rospy.loginfo("-------------------------------------------")
            if selection == "a":
                raise rospy.ROSInterruptException
            elif selection == "b":
                res = rospy.wait_for_message(
                    self.prefix + "/anydrive/reading", msg_defs.Reading
                )
                rospy.loginfo(
                    "Current: {}, Position: {}, Velocity: {}, Torque: {}".format(
                        res.state.current,
                        res.state.joint_position,
                        res.state.joint_velocity,
                        res.state.joint_torque,
                    )
                )
            elif selection == "c":
                msg = msg_defs.Command()
                value = input("Enter position: ")
                value = float(value)
                msg.mode.mode = 8
                msg.joint_position = value
                self.cmd_pub.publish(msg)
                rospy.loginfo("Command sent")
            elif selection == "d":
                msg = msg_defs.Command()
                value = input("Enter velocity: ")
                value = float(value)
                msg.mode.mode = 9
                msg.motor_velocity = value
                msg.gear_velocity = value
                msg.joint_velocity = value
                self.cmd_pub.publish(msg)
                rospy.loginfo("Command sent")
            rospy.sleep(0.5)

    def stop(self):
        rospy.loginfo("Stopping drive")
        msg = msg_defs.Command()
        msg.mode.mode = 1
        self.cmd_pub.publish(msg)


if __name__ == "__main__":
    cmd = ManualCommander()
    try:
        cmd.run()
    except rospy.ROSInterruptException:
        cmd.stop()
