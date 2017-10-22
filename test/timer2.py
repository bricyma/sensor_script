from timer import TestTimer
import rospy


class EsrDriverNode:
    def __init__(self):
        self.radarDriverModel = TestTimer()


if __name__ == '__main__':
    rospy.init_node('esr_driver_node', anonymous=True)
    a = EsrDriverNode()
    rospy.spin()
