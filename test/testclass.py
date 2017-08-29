import rospy
from std_msgs.msg import Int8


class TestTimer:
    def __init__(self):
        frequency1 = 20
        frequency2 = 50

        rospy.Subscriber('/topic1', Int8, self.recvtopic1)
        rospy.Subscriber('/topic2', Int8, self.recvtopic2)
        rospy.Timer(rospy.Duration(1.0 / frequency1), self.callback1)
        rospy.Timer(rospy.Duration(1.0 / frequency2), self.callback2)
        self.pub_pose1 = rospy.Publisher('/pose1', Int8, queue_size=1)
        self.pub_pose2 = rospy.Publisher('/pose2', Int8, queue_size=1)



        self.data1 = 1
        self.data2 = 2

    def recvtopic1(self, msg):
        self.data1 = msg.data

    def recvtopic2(self, msg):
        self.data2 = msg.data

    def callback1(self, event):
        self.pub_pose1.publish(self.data1)

    def callback2(self, event):
        self.pub_pose2.publish(self.data2)

if __name__ == "__main__":
    rospy.init_node('gps_points_generator', anonymous=True)
    _ = TestTimer()
    rospy.spin()






