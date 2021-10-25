#!/usr/bin/env python
from __future__ import print_function
import rospy
from std_msgs.msg import String
import tty, select, sys, termios
import time


def getKey(key_timeout, settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


listen_for = "abcdefghijklmnopqrstuvwxyz .,;':\"[]\{\}+\\=/?><-1234567890"
enter_key = "\x0d"
ctrl_c = "\x03"


def talker():
    rospy.init_node("lift_trigger", anonymous=True)
    pub = rospy.Publisher("lift_arm", String, queue_size=10)
    msg = String()

    settings = termios.tcgetattr(sys.stdin)

    while not rospy.is_shutdown():
        key = getKey(None, settings)
        if key in listen_for:
            print(key, end="")
            sys.stdout.flush()
            pub.publish(msg)
        elif key == enter_key:
            print("")
            sys.stdout.flush()
        elif key == ctrl_c:
            print("\nKeyboard interrupt")
            msg.data = "q"
            pub.publish(msg)
            break
        time.sleep(0.05)


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
