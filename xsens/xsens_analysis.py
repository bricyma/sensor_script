# python orientation.py *.bag
# author: Zhibei Ma
# date: 2017/11/27
# compare the GPS/IMU performane between Novatel propak6 and Xsens MTI-710

import matplotlib.pyplot as plt
import math as m
import rosbag
import sys
import numpy as np
import math
from llh2enu.llh2enu_gps_transformer import *
import tf
import copy

EARTH_RADIUS = 6378137.0


class XsensAnalysis:
    def __init__(self):
        self.bag_list = []
        self.bag_list.append(sys.argv[1])
        bag_path = '../../rosbag/xsens_analysis/'
        for i in range(0, len(self.bag_list)):
            self.bag_list[i] = bag_path + self.bag_list[i]

        self.diff = []   # gps yaw - inspvax
        self.data = {'n_raw': {}, 'x_raw': {}, 'novatel': {}, 'xsens': {}}
        self.transform = gps_transformer()
        self.baseLat = 32.339343  # Tucson
        self.baseLon = -111.008097
        self.raw_novatel = 'raw_novatel'

    def RAD2DEG(x):
        return x * (180 / np.pi)

    def readbag(self):
        key = ['lat', 'lon', 'x', 'y', 'yaw', 'pitch', 'roll', 't']
        for type in self.data:
            for k in key:
                self.data['n_raw'][k] = []

        first_flag = False  # choose the start point as the base, if True
        for bag in self.bag_list:
            bag = rosbag.Bag(bag)
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                if first_flag:
                    self.baseLat = msg.latitude
                    self.baseLon = msg.longitude
                    first_flag = False
                self.data['raw']['t'].append(
                    msg.header2.stamp.secs + msg.header2.stamp.nsecs / 1000000000.0)
                self.data['raw']['pitch'].append(msg.pitch)
                self.data['raw']['roll'].append(msg.roll)
                self.data['raw']['yaw'].append(msg.azimuth)
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data['raw']['x'].append(x)
                self.data['raw']['y'].append(y)

            for topic, msg, t in bag.read_messages(topics=['/xsens_driver/imupos']):
                self.data['xsens']['t'].append(
                    msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0)
                self.data['xsens']['yaw'].append(msg.yaw)
                self.data['xsens']['roll'].append(msg.roll)
                self.data['xsens']['pitch'].append(-msg.pitch)
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data['xsens']['x'].append(x)
                self.data['xsens']['y'].append(y)
            bag.close()

    # time sync
    def sync(self):
        self.data['novatel'] = copy.deepcopy(self.data['xsens'])
        count = 0
        for i in range(0, len(self.data['xsens']['t']) - 100):
            key = ['x', 'y', 'yaw', 'pitch', 'roll', 't']
            j, min_value = self.local_minimal(i)
            print min_value
            for k in key:
                if abs(min_value) < 0.05:  # cannot sync
                    self.data['novatel2'][k][count].append(self.data['raw'][k][j])
                    self.data['xsens2'][k][count].append(self.data['xsens'][k][j])
            count += 1

    def local_minimal(self, index):
        limit = 40
        min_value = 100
        min_index = index
        if index >= limit:
            for i in range(index - limit, index + limit):
                if abs(self.data['xsens']['t'][index] - self.data['raw']['t'][i]) < abs(min_value):
                    min_value = abs(self.data['xsens']['t']
                              [index] - self.data['raw']['t'][i])
                    min_index = i
        return min_index, min_value

    def compare(self):
        for brand in self.data:
            for key in self.data[brand]:
                self.data[brand][key] = np.array(self.data[brand][key])

        self.data['diff_x'] = np.array([])
        self.data['diff_y'] = np.array([])
        self.data['diff_pos'] = np.array([])

        self.data['diff_x'] = self.data['novatel']['x'] -\
            self.data['xsens']['x']
        self.data['diff_y'] = self.data['novatel']['y'] -\
            self.data['xsens']['y']

        self.data['diff_pos'] = np.sqrt(
            self.data['diff_x']**2 + self.data['diff_y']**2)
        print 'mean pos diff: ', np.mean(self.data['diff_pos'])

    def plot(self):
        plt.subplot(611)
        plt.plot(self.data['novatel']['t'] -
                 self.data['xsens']['t'], 'r', label='ros time diff')
        plt.legend(loc='upper left')
        plt.subplot(612)
        # plot the pitch diff between novatel and xsens
        plt.plot(self.data['novatel']['pitch'] -
                 self.data['xsens']['pitch'], 'r', label='pitch diff')
        plt.legend(loc='upper left')
        plt.subplot(613)
        plt.plot(self.data['novatel']['roll'] -
                 self.data['xsens']['roll'], 'r', label='roll diff')
        plt.legend(loc='upper left')
        plt.subplot(614)
        plt.plot(self.data['novatel']['yaw'] -
                 self.data['xsens']['yaw'], 'r', label='yaw diff')
        plt.legend(loc='upper left')
        plt.subplot(615)

        plt.plot(self.data['novatel']['x'], self.data['novatel']
                 ['y'], 'ro', label='novatel path')
        plt.plot(self.data['xsens']['x'], self.data['xsens']
                 ['y'], 'bo', label='xsens path')
        plt.legend(loc='upper left')
        plt.subplot(616)
        plt.plot(self.data['novatel']['x'] - self.data['xsens']['x'], 'r',
                 label='East')
        plt.plot(self.data['novatel']['y'] - self.data['xsens']['y'], 'b',
                 label='North')
        plt.legend(loc='upper left')
        plt.show()

    def run(self):
        self.readbag()
        self.sync()
        self.compare()
        self.plot()


if __name__ == '__main__':
    # bag = sys.argv[1]
    ori = XsensAnalysis()
    ori.run()
