# python orientation.py *.bag
# author: Zhibei Ma
# date: 2017/06/29
# check the offset angle between imu and vehicle's yaw

import matplotlib.pyplot as plt
import rosbag
import sys
import numpy as np
import math

EARTH_RADIUS = 6378137.0


class Orientation:
    def __init__(self, bagname):
        self.bag = rosbag.Bag(bagname)
        self.readbag()
        self.plot()

    def readbag(self):
        self.data = {'insspd': [], 'inspvax':[]}
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/insspd']):
            self.data['insspd'].append(msg.track_ground)
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            self.data['inspvax'].append(msg.azimuth)
        self.bag.close()
        self.data['insspd'] = np.array(self.data['insspd'])
        self.data['inspvax'] = np.array(self.data['inspvax'])


    def plot(self):
        plt.subplot(211)
        plt.plot(self.data['insspd'], 'r', label='insspd')
        plt.plot(self.data['inspvax'], 'b', label='inspvax')
        plt.legend(loc='upper left')
        plt.subplot(212)
        size = min(len(self.data['inspvax']), len(self.data['insspd']))

        diff = self.data['inspvax'][0:size] - self.data['insspd'][0:size]
        print 'mean of (inspvax - insspd): ', np.mean(diff)
        plt.plot(diff, 'r', label = 'inspvax- insspd')
        plt.legend(loc='upper left')
        plt.show()

if __name__ == '__main__':
    bag = sys.argv[1]
    ori = Orientation(bag)
