import rospy
from std_msgs.msg import Int8

class TestTimer(object):
    def __init__(self):
        self.speed = 0
        self.yaw_rate = 0

        listen_rate = 20
        send_rate = 50
        rospy.Timer(rospy.Duration(1.0 / listen_rate), self.listenCAN)
        rospy.Timer(rospy.Duration(1.0 / send_rate), self.sendCAN)

        rospy.Subscriber('/topic1', Int8, self.recvVelocity)
        rospy.Subscriber('/topic2', Int9, self.recvYawRate)
        self.pub = rospy.Publisher('/topic3', Int8, queue_size = 10)


    def listenCAN(self, event):
        print 'listen', 1
        self.pub.publish(14)

    def sendCAN(self, event):
        print 'send', self.speed, self.yaw_rate


    def recvVelocity(self, msg):
        self.speed = msg.data

    def recvYawRate(self, msg):
        self.yaw_rate = msg.data


if __name__ == "__main__":
    # rospy.init_node('esr_driver_node', anonymous=True)
    # t = TestTimer()
    rospy.spin()
