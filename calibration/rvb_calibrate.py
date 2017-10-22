import matplotlib.pyplot as plt
import rosbag
import sys
import numpy as np

class Calibrate:
    def __init__(self, bagname):
        bag_path = '../../rosbag/'
        bag = bag_path + bagname
        self.bag = rosbag.Bag(bag)
        self.yaw_diff = []  # vehicle yaw - imu yaw

    def read_bag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/calibrate/diff']):
            if abs(msg.data) < 2:
                self.yaw_diff.append(msg.data)

    def plot(self):
        plt.plot(self.yaw_diff, 'r', label='yaw offset')
        plt.legend(loc='upper left')
        plt.show()

    def run(self):
        self.read_bag()
        print 'average yaw angle offset: ', np.mean(self.yaw_diff)
        self.plot()


if __name__ == '__main__':
    bag = sys.argv[1]
    cal = Calibrate(bag)
    cal.run()

