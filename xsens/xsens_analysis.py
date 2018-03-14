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
        self.data = {'novatel':{}, 'xsens':{}, 'raw':{}}
        self.transform = gps_transformer()
        self.baseLat = 32.339343  # Tucson
        self.baseLon = -111.008097
        self.raw_novatel = 'raw_novatel'

    def RAD2DEG(x):
        return x * (180 / np.pi)

    def readbag(self):
        key = ['lat', 'lon', 'x', 'y', 'yaw', 'pitch', 'roll', 't']
        for k in key:
            self.data['raw'][k] = []
            self.data['xsens'][k] = []

        first_flag = False  # choose the start point as the base, if True
        for bag in self.bag_list:
            bag = rosbag.Bag(bag)
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                if first_flag:
                    self.baseLat = msg.latitude
                    self.baseLon = msg.longitude
                    first_flag = False
                self.data['raw']['t'].append(msg.header2.stamp.secs + msg.header2.stamp.nsecs/1000000000.0)
                self.data['raw']['pitch'].append(msg.pitch)
                self.data['raw']['roll'].append(msg.roll)
                self.data['raw']['yaw'].append(msg.azimuth)
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data['raw']['x'].append(x)
                self.data['raw']['y'].append(y)

            for topic, msg, t in bag.read_messages(topics=['/xsens_driver/imupos']):
                self.data['xsens']['t'].append(msg.header.stamp.secs + msg.header.stamp.nsecs/1000000000.0)
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
        for i in range(0, len(self.data['xsens']['t']) - 100):
            key = ['x', 'y', 'yaw', 'pitch', 'roll', 't']
            j = self.local_minimal(i)
            for k in key:
                self.data['novatel'][k][i] = self.data['raw'][k][j]

            novatel_x = self.data['novatel']['x'][i]
            novatel_y = self.data['novatel']['y'][i]
            xsens_x = self.data['xsens']['x'][i]
            xsens_y = self.data['xsens']['y'][i]
            novatel_t = self.data['novatel']['t'][i]
            if i < 90000:
                print novatel_t, novatel_x, xsens_x, novatel_x - xsens_x, 'y: ', novatel_y, xsens_y, novatel_y - xsens_y


    def local_minimal(self, index):
        limit = 40
        min = 100
        min_index = index
        if index >= limit:
            for i in range(index-limit, index+limit):
                if abs(self.data['xsens']['t'][index] - self.data['raw']['t'][i]) < min:
                    min = abs(self.data['xsens']['t'][index] - self.data['raw']['t'][i])
                    min_index = i
        return min_index


    def compare(self):
        self.data['diff_x'] = np.array([])
        self.data['diff_y'] = np.array([])
        self.data['diff_pos'] = np.array([])
        self.data['novatel']['x'] = np.array(self.data['novatel']['x'])
        self.data['novatel']['y'] = np.array(self.data['novatel']['y'])
        self.data['xsens']['x'] = np.array(self.data['xsens']['x'])
        self.data['xsens']['y'] = np.array(self.data['xsens']['y'])

        self.data['diff_x'] = self.data['novatel']['x'] -\
            self.data['xsens']['x']
        self.data['diff_y'] = self.data['novatel']['y'] -\
            self.data['xsens']['y']

        self.data['diff_pos'] = np.sqrt(
            self.data['diff_x']**2 + self.data['diff_y']**2)
        print 'mean pos diff: ', np.mean(self.data['diff_pos'])

    def plot(self):
        plt.subplot(611)
        plt.plot(self.data['xsens']['t'], 'b', label='xsens time')
        plt.plot(self.data['novatel']['t'], 'r', label='novatal time')
        # plt.plot(self.data['novatel']['pitch'], 'r', label='novatal pitch')
        # plt.plot(self.data['xsens']['pitch'], 'b', label='xsens pitch')
        plt.legend(loc='upper left')
        plt.subplot(612)
        # plot the pitch diff between novatel and xsens
        # plt.plot(self.data['novatel']['pitch'] - self.data['xsens']
                 # ['pitch'], 'r', label='pitch diff between novatel and xsens')
        plt.plot(self.data['novatel']['roll'], 'r', label='novatal roll')
        plt.plot(self.data['xsens']['roll'], 'b', label='xsens roll')
        plt.legend(loc='upper left')
        plt.subplot(613)
        plt.plot(self.data['novatel']['yaw'], 'r', label='novatel yaw')
        plt.plot(self.data['xsens']['yaw'], 'b', label='xsens yaw')
        plt.legend(loc='upper left')
        plt.subplot(614)
        # plt.plot(self.data['diff'], 'r', label='novatel yaw - xsens yaw')
        plt.legend(loc='upper left')
        plt.subplot(615)

        plt.plot(self.data['novatel']['x'], self.data['novatel']
                 ['y'], 'r', label='novatel path')
        plt.plot(self.data['xsens']['x'], self.data['xsens']
                 ['y'], 'b', label='xsens path')
        plt.legend(loc='upper left')
        plt.subplot(616)
        plt.plot(self.data['diff_pos'], 'r',
                 label='pos diff between novatel and xsens')
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
