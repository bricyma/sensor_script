# author: Zhibei Ma
# date: 2018/03/20
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

        self.corr = []   # time correction value for ros time
        self.data = {'raw': {}, 'xsens': {}, 'novatel': {}}
        self.transform = gps_transformer()
        self.baseLat = 32.339343  # Tucson
        self.baseLon = -111.008097
        self.raw_novatel = 'raw_novatel'

    def RAD2DEG(x):
        return x * (180 / np.pi)

    def readbag(self):
        key = ['lat', 'lon', 'x', 'y', 'yaw', 'pitch', 'roll', 't_gps', 't_ros']
        for t in self.data:
            for k in key:
                self.data[t][k] = []

        first_flag = False  # choose the start point as the base, if True
        for bag in self.bag_list:
            bag = rosbag.Bag(bag)
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                if first_flag:
                    self.baseLat = msg.latitude
                    self.baseLon = msg.longitude
                    first_flag = False
                novatel_time = 315964800 + msg.header.gps_week * 7 * 24 * 60 * 60 \
                + float(msg.header.gps_week_seconds) / \
                1000 - 18  # GPS time to epoch second
                pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
                
                self.data['raw']['t_gps'].append(novatel_time)
                # self.data['raw']['t_ros'].append(pc_time)
                # choose the right data through the visualization plot of the difference 
                # between pc_time and novatel_time
                self.data['raw']['t_ros'].append(pc_time)
                # print novatel_time - pc_time
                self.data['raw']['pitch'].append(msg.pitch)
                self.data['raw']['roll'].append(msg.roll)
                self.data['raw']['yaw'].append(msg.azimuth)
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data['raw']['x'].append(x)
                self.data['raw']['y'].append(y)

                self.data['raw']['lat'].append(msg.latitude)
                self.data['raw']['lon'].append(msg.longitude)

            for topic, msg, t in bag.read_messages(topics=['/xsens_driver/imupos']):
                self.data['xsens']['t_ros'].append(
                    msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000.0)
                self.data['xsens']['yaw'].append(msg.yaw)
                self.data['xsens']['roll'].append(msg.roll)
                self.data['xsens']['pitch'].append(-msg.pitch)
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data['xsens']['x'].append(x)
                self.data['xsens']['y'].append(y)
            bag.close()
        self.time_correction()


    def time_correction(self):
        for i in range(0, len(self.data['raw']['t_ros'])):
            self.corr.append(self.filter(i))

        for i in range(0, len(self.data['raw']['t_ros'])):
            self.data['raw']['t_ros'][i] = self.data['raw']['t_gps'][i] - self.corr[i]


    def filter(self, n):
        window = 50
        upper = window
        lower = -window
        dt_sum = 0
        if n <= window:
            return 0
        if n > len(self.data['raw']['t_ros']) - window:
            upper = 0
        for i in range(lower, upper):
            diff = self.data['raw']['t_gps'][n+i] - self.data['raw']['t_ros'][n+i]
            dt_sum += diff
        return dt_sum / (upper - lower)     

    # time sync
    def sync(self):
        self.data['novatel'] = copy.deepcopy(self.data['xsens'])
        for i in range(0, len(self.data['xsens']['t_ros']) - 200):
            key = ['x', 'y', 'yaw', 'pitch', 'roll', 't_ros']
            j, min_value = self.local_minimal(i)
            for k in key:
                if abs(min_value) < 0.05:  
                    self.data['novatel'][k][i] = self.data['raw'][k][j]
        for brand in self.data:
            for key in self.data[brand]:
                self.data[brand][key] = np.array(self.data[brand][key])

    def local_minimal(self, index):
        limit = 80
        min_value = 100
        min_index = index
        if index >= limit:
            for i in range(index - limit, index + limit):
                if abs(self.data['xsens']['t_ros'][index] - self.data['raw']['t_ros'][i]) < abs(min_value):
                    min_value = abs(self.data['xsens']['t_ros'][index] - self.data['raw']['t_ros'][i])
                    min_index = i
        return min_index, min_value


    def plot(self):
        # time  
        plt.subplot(311)
        # delta_t = self.data['novatel']['t_ros'] - self.data['xsens']['t_ros']
        # plt.plot(self.data['novatel']['t_ros'], 'r', label = 'novatel t')
        plt.plot(self.data['raw']['t_ros'], 'g', label = 'raw t')
        plt.plot(self.data['xsens']['t_ros'], 'b', label = 'xsens t')
        plt.legend(loc='upper left')

        plt.subplot(312)
        delta_t = self.data['raw']['t_gps'] - self.data['raw']['t_ros']
        plt.plot(delta_t, 'r', label = 't_gps - t_ros')
        plt.legend(loc='upper left')
        
        plt.subplot(313)
        delta_t = self.data['raw']['t_gps'] - self.data['raw']['t_ros']
        plt.plot(delta_t, 'r', label = 't_gps - t_ros')
        plt.legend(loc='upper left')
        plt.show()

        # orientation
        plt.subplot(611)
        plt.plot(self.data['raw']['pitch'], 'r', label = 'novatel pitch')
        plt.plot(self.data['xsens']['pitch'], 'b', label = 'xsens pitch')
        plt.legend(loc='upper left')

        plt.subplot(612)
        delta = self.data['novatel']['pitch'] - self.data['xsens']['pitch']
        self.plot_content(delta, 'pitch accuracy')

        plt.subplot(613)
        plt.plot(self.data['novatel']['pitch'], 'r', label = 'novatel pitch')
        plt.plot(self.data['xsens']['pitch'], 'b', label = 'xsens pitch')
        plt.legend(loc='upper left')

        plt.subplot(614)
        delta = self.data['novatel']['roll'] - self.data['xsens']['roll']
        self.plot_content(delta, 'roll accuracy')
        
        plt.subplot(615)
        plt.plot(self.data['raw']['yaw'], 'r', label = 'novatel yaw')
        plt.plot(self.data['xsens']['yaw'], 'b', label = 'xsens yaw')
        plt.legend(loc='upper left')

        plt.subplot(616)
        delta = self.data['novatel']['yaw'] - self.data['xsens']['yaw']
        self.plot_content(delta, 'yaw accuracy')
        
        plt.show()
        # position
        plt.subplot(411)
        delta = self.data['novatel']['x'] - self.data['xsens']['x']
        self.plot_content(delta, 'x accuracy')

        plt.subplot(412)
        delta = self.data['novatel']['y'] - self.data['xsens']['y']
        self.plot_content(delta, 'y accuracy')




        plt.show()


    def plot_content(self, delta, _label):
        plt.plot(delta, 'r', label = _label)
        plt.legend(loc='upper left')


    def run(self):
        self.readbag()
        self.sync()
        self.plot()


if __name__ == '__main__':
    # bag = sys.argv[1]
    ori = XsensAnalysis()
    ori.run()
