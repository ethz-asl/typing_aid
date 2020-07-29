import rospy
from std_msgs.msg import Empty

if __name__=='__main__':
    pub = rospy.Publisher('lifting_trigger', Empty, queue_size=10)
    rospy.init_node('talkerrrr', anonymous=True)
    while not rospy.is_shutdown():
        raw_input("Press to publish ...")
        pub.publish(Empty())
