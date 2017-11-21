# python orientation.py *.bag
# author: Zhibei Ma
# date: 2017/06/29
# check the offset angle between imu and vehicle's yaw

import matplotlib.pyplot as plt
import math as m
import rosbag
import sys
import numpy as np
import math
EARTH_RADIUS = 6378137.0

class Orientation:
    def __init__(self, bagname):
        bag_path = ''
        bag = bag_path + bagname
        self.bag = rosbag.Bag(bag)
        self.diff_age = []


    def readbag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/rtkpos']):
            self.diff_age.append(msg.diff_age)
        self.bag.close()


    def plot(self):
        plt.subplot(211)
        plt.subplot(212)
        plt.plot(self.diff_age, 'r', label='diff')
        plt.show()


    def run(self):
        self.readbag()
        self.plot()


if __name__ == '__main__':
    bag = sys.argv[1]
    ori = Orientation(bag)
    ori.run()
