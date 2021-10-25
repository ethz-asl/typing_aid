import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher("lift_arm", String, queue_size=10)
    rospy.init_node("lift_trigger", anonymous=True)
    msg = String()

    while not rospy.is_shutdown():
        inp = raw_input("Enter to lift.")
        msg.data = inp
        pub.publish(msg)


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
