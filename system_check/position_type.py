## Plot the Novatel BESTPOS's position type and position accuracy together
import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np


class PosType:
    def __init__(self):
        bag_path = '../../rosbag/'
        self.bagname = []
        self.bagname.append(bag_path + sys.argv[1])
        self.bagname.append(bag_path + sys.argv[2])
        self.bag = []
        for bag in self.bagname:
            self.bag.append(rosbag.Bag(bag))
        self.postype1 = [] # bag1 position type
        self.posacc1 = []  # bag1 position accuracy
        self.postype2 = [] # bag2 position type
        self.posacc2 = []  # bag2 position accuracy

        self.altitude_std1 = [] # bag1 altitude standard accuracy
        self.altitude_std2 = []  # bag2 altitude standard accuracy

        self.satellite_used = []  # satellites used in solution
        self.satellite1 = []
        self.satellite2 = []
        self.index = [0, 0]
    def readbag(self, flag):
        # flag == 1, bestpos
        # flag == 0, inspvax

        count = 0
        for bag in self.bag:
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/bestpos']):
                if count == 0:
                    self.satellite1.append(msg.sol_svs)
                else:
                    self.satellite2.append(msg.sol_svs)

            for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                self.index[count] += 1
                if count == 0:
                    if self.index[0] > 15000:
                        self.posacc1.append(m.sqrt(msg.latitude_std ** 2 + msg.longitude_std ** 2))
                        self.postype1.append(msg.position_type)
                        self.altitude_std1.append(msg.altitude_std)
                else:
                    if self.index[1] > 15000:
                        self.posacc2.append(m.sqrt(msg.latitude_std ** 2 + msg.longitude_std ** 2))
                        self.postype2.append(msg.position_type)
                        self.altitude_std2.append(msg.altitude_std)

            count += 1
            bag.close()

    def plot(self):
        plt.subplot(411)
        plt.plot(self.postype1, 'r')
        plt.plot(self.postype2, 'b')

        plt.title("Position Type")

        plt.subplot(412)
        plt.plot(self.posacc1, 'r', label='bag1 2d position accuracy')
        plt.plot(self.posacc2, 'b', label='bag2 2d position accuracy')
        # plt.plot([np.mean(self.posacc1)] * len(self.posacc1), 'r', linestyle='-', linewidth=3, label='average')
        # plt.plot([np.mean(self.posacc2)] * len(self.posacc2), 'r', linestyle='-', linewidth=3, label='average')
        plt.title("INSPVAX 2d Horizontal Position Accuracy")
        plt.legend(loc='upper left')
        plt.subplot(413)
        plt.plot(self.altitude_std1, 'r', label='bag1 vertical position accuracy')
        plt.plot(self.altitude_std2, 'b', label='bag2 vertical position accuracy')
        plt.legend(loc='upper left')
        plt.title("INSPVAX Vertical Position Accuracy")

        plt.subplot(414)
        plt.plot(self.satellite1, 'r', label='bag1 satellites')
        plt.plot(self.satellite2, 'b', label='bag2 satellites')
        plt.legend(loc='upper left')
        plt.title("Satellites used in solution")

        plt.show()

if __name__ == '__main__':
    pos = PosType()
    pos.readbag(0)
    print 'bag1: average satellites', np.mean(pos.satellite1)
    print 'bag1: average horizontal position accuracy', np.mean(pos.posacc1)
    print 'bag1: standard deviation of 2d position accuracy', np.std(pos.posacc1)
    print 'bag1: average vertical position accuracy', np.mean(pos.altitude_std1)

    print 'bag2: average satellites', np.mean(pos.satellite2)
    print 'bag2: average horizontal position accuracy', np.mean(pos.posacc2)
    print 'bag2: standard deviation of 2d position accuracy', np.std(pos.posacc2)
    print 'bag2: average vertical position accuracy', np.mean(pos.altitude_std2)

    pos.plot()
