import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node('esr_node2', anonymous=True)

    pub = rospy.Publisher('esr_front', String, queue_size=10)
    r = rospy.Rate(100)
    i = 0
    while not rospy.is_shutdown():
        # print i
        # i += 1
        pub.publish('aaa')
        r.sleep()
    rospy.spin()
