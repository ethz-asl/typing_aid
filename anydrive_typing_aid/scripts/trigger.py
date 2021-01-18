import rospy
from std_msgs.msg import String, Empty

def talker():
    pub = rospy.Publisher('lift_arm', Empty, queue_size=10)
    rospy.init_node('lift_trigger', anonymous=True)

    while not rospy.is_shutdown():
        _ = raw_input("Enter to lift.")
        pub.publish()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass