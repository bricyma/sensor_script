# python orientation.py *.bag
# author: Zhibei Ma
# date: 2017/06/29

import matplotlib.pyplot as plt
import math as m
import rosbag
import sys
import numpy as np


class Orientation:
    def __init__(self):
        bag_path = '../../rosbag/'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.north_vel= []
        self.east_vel = []
        self.yaw = []
        self.ori_vel = []  # orientation calculated from east_velocity and north_velocity
        self.diff = []   # yaw - orientation_from

    def readbag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            self.north_vel.append(msg.north_velocity)
            self.east_vel.append(msg.east_velocity)
            self.yaw.append(msg.azimuth)
        self.bag.close()

    def orientation_from_velocity(self):
        for i in range(0, len(self.north_vel)):
            if np.arctan2(self.east_vel[i], self.north_vel[i]) < 0:
                cur_ori = np.arctan2(self.east_vel[i], self.north_vel[i]) * 180 / np.pi + 360
            else:
                cur_ori = np.arctan2(self.east_vel[i], self.north_vel[i]) * 180 / np.pi

            self.ori_vel.append(cur_ori)

    def compare(self):
        for i in range(0, len(self.north_vel)):
            if self.yaw[i] - self.ori_vel[i] > 100:
                print self.yaw[i], self.ori_vel[i]
            self.diff.append(self.yaw[i] - self.ori_vel[i])

    def plot(self):
        plt.subplot(211)
        plt.plot(self.yaw, 'r', label='yaw')
        plt.plot(self.ori_vel, 'b', label='orientation from velocity')
        plt.legend(loc='upper left')
        plt.subplot(212)
        plt.plot(self.diff, 'r', label='diff')
        plt.legend(loc='upper left')

        plt.show()

    def output(self):
        print len(self.yaw), len(self.ori_vel)
        print 'mean of diff: ', np.round(np.mean(self.diff), 4)

if __name__ == '__main__':
    ori = Orientation()
    ori.readbag()
    ori.orientation_from_velocity()
    ori.compare()
    ori.output()
    ori.plot()
