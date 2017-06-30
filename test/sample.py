# parse sample file
# author: Zhibei Ma
# date: 2017/06/29
import matplotlib.pyplot as plt
import rosbag
import sys


class Orientation:
    def __init__(self):
        bag_path = '../../rosbag/'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.north_vel= []

    def readbag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            self.north_vel.append(msg.north_velocity)
        self.bag.close()

    def plot(self):
        plt.plot(self.north_vel, 'r', label='north vel')
        plt.legend(loc='upper left')
        plt.show()

if __name__ == '__main__':
    ori = Orientation()
    ori.readbag()
    ori.plot()
