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

EARTH_RADIUS = 6378137.0

class XsensAnalysis:
    def __init__(self):
        self.bag_list = []
        self.bag_list.append(sys.argv[1])
        bag_path = '../../rosbag/xsens_analysis/'
        for i in range(0, len(self.bag_list)):
            self.bag_list[i] = bag_path + self.bag_list[i]

        self.diff = []   # gps yaw - inspvax
        self.data = {'novatel':{}, 'xsens':{}}
        self.transform = gps_transformer()
        self.baseLat = 32.339343  # Tucson
        self.baseLon = -111.008097

    def RAD2DEG(x):
        return x * (180 / np.pi)

    def readbag(self):
        self.data['novatel'] = {}
        self.data['xsens'] = {}
        key = ['lat', 'lon', 'x', 'y', 'yaw', 'pitch', 'roll', 'vel_x', 'vel_y', 't']
        for k in key:
            self.data['novatel'][k] = []
            self.data['xsens'][k] = []

        first_flag = False  # choose the start point as the base, if True
        for bag in self.bag_list:
            bag = rosbag.Bag(bag)
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                if first_flag:
                    self.baseLat = msg.latitude
                    self.baseLon = msg.longitude
                    first_flag = False
                self.data['novatel']['t'].append(msg.header2.stamp.secs + msg.header2.stamp.nsecs/1000000000.0)
                self.data['novatel']['pitch'].append(msg.pitch)
                self.data['novatel']['roll'].append(msg.roll)
                self.data['novatel']['yaw'].append(msg.azimuth)
                self.data['novatel']['lat'].append(msg.latitude)
                self.data['novatel']['lon'].append(msg.longitude)
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data['novatel']['x'].append(x)
                self.data['novatel']['y'].append(y)

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

    def compare(self):
        self.data['diff'] = np.array([])
        self.data['diff_x'] = np.array([])
        self.data['diff_y'] = np.array([])
        self.data['diff_pos'] = np.array([])
        self.data['novatel']['yaw'] = np.array(self.data['novatel']['yaw'])
        self.data['xsens']['yaw'] = np.array(self.data['xsens']['yaw'])

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
        plt.subplot(611)
        plt.plot(self.data['novatel']['pitch'], 'r', label='novatal pitch')
        plt.plot(self.data['xsens']['pitch'], 'b', label='xsens pitch')
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
        plt.plot(self.data['diff'], 'r', label='novatel yaw - xsens yaw')
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
        self.compare()

        self.plot()


if __name__ == '__main__':
    # bag = sys.argv[1]
    ori = XsensAnalysis()
    ori.run()
