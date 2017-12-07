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

EARTH_RADIUS = 6378137.0


class XsensAnalysis:
    def __init__(self, bagname):
        bag_path = ''
        bag = bag_path + bagname
        self.bag = rosbag.Bag(bag)
        self.diff = []   # gps yaw - inspvax
        self.data = {}
        self.transform = gps_transformer()

        self.baseLat = 39.03775082210  # wuhudao
        self.baseLon = 118.43091220755

        # self.baseLat = 31.344678  # shanghai
        # self.baseLon = 121.379186

        # self.baseLat = 39.714178  # beijing
        # self.baseLon = 117.305466

        # self.baseLat = 32.694052  # san diego
        # self.baseLon = -113.958389

        print 'baseLat: ', self.baseLat
        print 'baseLon: ', self.baseLon

    def readbag(self):
        self.data['novatel'] = {}
        self.data['xsens'] = {}
        key = ['lat', 'lon', 'x', 'y', 'yaw']
        for k in key:
            self.data['novatel'][k] = []
            self.data['xsens'][k] = []

        first_flag = False  # choose the start point as the base, if True
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            if first_flag:
                self.baseLat = msg.latitude
                self.baseLon = msg.longitude
                first_flag = False
            self.data['novatel']['yaw'].append(msg.azimuth)
            self.data['novatel']['lat'].append(msg.latitude)
            self.data['novatel']['lon'].append(msg.longitude)
            x, y = self.transform.llh2enu_5(
                msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
            self.data['novatel']['x'].append(x)
            self.data['novatel']['y'].append(y)

        for topic, msg, t in self.bag.read_messages(topics=['/xsens_driver/imupos']):
            msg.yaw = msg.yaw * -1 + 90
            if msg.yaw < 0:
                msg.yaw += 360
            self.data['xsens']['yaw'].append(msg.yaw)
            x, y = self.transform.llh2enu_5(
                msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
            self.data['xsens']['x'].append(x)
            self.data['xsens']['y'].append(y)

        self.bag.close()

    def compare(self):
        self.data['diff'] = np.array([])
        self.data['diff_x'] = np.array([])
        self.data['diff_y'] = np.array([])
        self.data['diff_pos'] = np.array([])
        self.data['novatel']['yaw'] = np.array(self.data['novatel']['yaw'])
        self.data['xsens']['yaw'] = np.array(self.data['xsens']['yaw'])


        for i in range(0, len(self.data['novatel']['yaw'])):
            if self.data['novatel']['yaw'][i] < 10:
                self.data['novatel']['yaw'][i] += 360


        # list to numpy array
        if len(self.data['novatel']['yaw']) < len(self.data['xsens']['yaw']):
            size = len(self.data['novatel']['yaw'])
        else:
            size = len(self.data['xsens']['yaw'])
        for sensor in self.data:
            for key in self.data[sensor]:
                self.data[sensor][key] = np.array(
                    self.data[sensor][key])[0:size]

        self.data['diff'] = self.data['novatel']['yaw'] - \
            self.data['xsens']['yaw']
        # self.data['diff'][self.data['diff'] > 15] = 0
        # self.data['diff'][self.data['diff'] < -15] = 0

        self.data['diff_x'] = self.data['novatel']['x'] -\
            self.data['xsens']['x']
        self.data['diff_y'] = self.data['novatel']['y'] -\
            self.data['xsens']['y']

        print 'novatel x: ', self.data['novatel']['x']

        print self.data['diff_x']

        self.data['diff_pos'] = np.sqrt(
            self.data['diff_x']**2 + self.data['diff_y']**2)
        print 'mean diff: ', np.mean(self.data['diff'])
        print 'mean pos diff: ', np.mean(self.data['diff_pos'])

    def plot(self):
        plt.subplot(411)
        plt.plot(self.data['novatel']['yaw'], 'r', label='novatel yaw')
        plt.plot(self.data['xsens']['yaw'], 'b', label='xsens yaw')
        plt.legend(loc='upper left')
        plt.subplot(412)
        plt.plot(self.data['diff'], 'r', label='novatel yaw - xsens yaw')
        plt.legend(loc='upper left')
        plt.subplot(413)
        plt.plot(self.data['novatel']['x'], self.data['novatel']
                 ['y'], 'r', label='novatel path')
        plt.plot(self.data['xsens']['x'], self.data['xsens']
                 ['y'], 'b', label='xsens path')
        plt.legend(loc='upper left')
        plt.subplot(414)
        plt.plot(self.data['diff_pos'], 'r',
                 label='pos diff between novatel and xsens')
        plt.legend(loc='upper left')
        plt.show()

    def run(self):
        self.readbag()
        self.compare()

        self.plot()


if __name__ == '__main__':
    bag = sys.argv[1]
    ori = XsensAnalysis(bag)
    ori.run()
