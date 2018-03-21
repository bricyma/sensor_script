# author: Zhibei Ma
# date: 2018/03/20
# compare the GPS/IMU performane between Novatel propak6 and Xsens MTI-710

import matplotlib
import matplotlib.pyplot as plt
import math as m
import rosbag
import sys
import numpy as np
import math
from llh2enu.llh2enu_gps_transformer import *
import tf
import copy
from statsmodels.nonparametric.smoothers_lowess import lowess
EARTH_RADIUS = 6378137.0


class XsensAnalysis:
    def __init__(self):
        self.bag_list = []
        self.bag_list.append(sys.argv[1])
        bag_path = '../../rosbag/'
        for i in range(0, len(self.bag_list)):
            self.bag_list[i] = bag_path + self.bag_list[i]

        self.corr = []   # time correction value for ros time
        # xsens2 is the updated xsens data with time sync
        self.data = {'raw': {}, 'xsens': {}, 'novatel': {}, 'xsens2': {}}
        self.transform = gps_transformer()
        self.baseLat = 32.339343  # Tucson
        self.baseLon = -111.008097
        self.raw_novatel = 'raw_novatel'

    def RAD2DEG(x):
        return x * (180 / np.pi)

    def readbag(self):
        key = ['lat', 'lon', 'x', 'y', 'yaw', 'pitch',
               'roll', 't_gps', 't_ros', 'vel_e', 'vel_n', 'vel']
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
                self.data['raw']['vel'].append(
                    np.sqrt(msg.east_velocity ** 2 + msg.north_velocity ** 2))

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
                self.data['xsens']['vel_e'].append(msg.vx)
                self.data['xsens']['vel_n'].append(msg.vy)
                self.data['xsens']['vel'].append(
                    np.sqrt(msg.vx ** 2 + msg.vy ** 2))

            bag.close()
        self.time_correction()

    # calculate the time diff between pc_time and gps_time, if there is no chronyc time sync, the time diff will be larger and larger.
    def time_correction(self):
        t = []
        for i in range(0, len(self.data['raw']['t_ros'])):
            self.corr.append(self.filter(i))
            t.append(i)
        print 'bb'
        # filtered = lowess(self.corr, t, is_sorted=True, frac=25, it=0)
        filtered = self.savitzky_golay(np.array(self.corr), 5001, 3)
        plt.plot(filtered, 'r', label='filtered')
        plt.plot(self.corr, 'b', label='corr')
        plt.legend(loc='upper left')
        plt.show()
        print 'aa'
        for i in range(0, len(self.data['raw']['t_ros'])):
            self.data['raw']['t_ros'][i] = self.data['raw']['t_gps'][i] - \
                filtered[i]  # self.corr[i]

    def savitzky_golay(self, y, window_size, order, deriv=0, rate=1):
        from math import factorial

        try:
            window_size = np.abs(np.int(window_size))
            order = np.abs(np.int(order))
        except ValueError, msg:
            raise ValueError("window_size and order have to be of type int")
        if window_size % 2 != 1 or window_size < 1:
            raise TypeError("window_size size must be a positive odd number")
        if window_size < order + 2:
            raise TypeError(
                "window_size is too small for the polynomials order")
        order_range = range(order + 1)
        half_window = (window_size - 1) // 2
        # precompute coefficients
        b = np.mat([[k**i for i in order_range]
                    for k in range(-half_window, half_window + 1)])
        m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
        # pad the signal at the extremes with
        # values taken from the signal itself
        firstvals = y[0] - np.abs(y[1:half_window + 1][::-1] - y[0])
        lastvals = y[-1] + np.abs(y[-half_window - 1:-1][::-1] - y[-1])
        y = np.concatenate((firstvals, y, lastvals))
        return np.convolve(m[::-1], y, mode='valid')

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
            diff = self.data['raw']['t_gps'][n + i] - \
                self.data['raw']['t_ros'][n + i]
            dt_sum += diff
        average = dt_sum
        return dt_sum / (upper - lower)

    # time sync
    # for each novatel time stamp, we will have one xsens time stamp

    def sync(self):
        self.data['novatel'] = copy.deepcopy(self.data['raw'])
        self.data['xsens2'] = copy.deepcopy(self.data['raw'])
        for i in range(0, len(self.data['novatel']['t_ros']) - 200):
            key = ['x', 'y', 'yaw', 'pitch', 'roll', 't_ros', 'vel']
            j, min_value = self.local_minimal(i)
            for k in key:
                self.data['xsens2'][k][i] = self.data['xsens'][k][j]
        for i in range(len(self.data['novatel']['t_ros']) - 200, len(self.data['novatel']['t_ros'])):
            key = ['x', 'y', 'yaw', 'pitch', 'roll', 't_ros', 'vel']
            for k in key:
                self.data['xsens2'][k][i] = self.data['xsens'][k][i]

        for brand in self.data:
            for key in self.data[brand]:
                self.data[brand][key] = np.array(self.data[brand][key])

        self.data['xsens'] = {}
        self.data['xsens'] = copy.deepcopy(self.data['xsens2'])


    def local_minimal(self, index):
        limit = 80
        min_value = 100
        min_index = index
        if index >= limit:
            for i in range(index - limit, index + limit):
                if abs(self.data['novatel']['t_ros'][index] - self.data['xsens']['t_ros'][i]) < abs(min_value):
                    min_value = abs(
                        self.data['novatel']['t_ros'][index] - self.data['xsens']['t_ros'][i])
                    min_index = i
        return min_index, min_value

    def analyze(self):
        for k in self.data['novatel']:
            delta = self.data['novatel'][k] - self.data['xsens'][k]
            if k == 'yaw' or 'roll' or 'pitch':
                delta -= np.mean(delta)
            print 'xsens ' + k + ' RMS: ', np.sqrt(np.mean(delta**2))
    

    def plot(self):
        # enlarge the font size of figure
        font = {'family': 'normal',
                'weight': 'bold',
                'size': 22}

        matplotlib.rc('font', **font)
        # time
        plt.subplot(311)
        # delta_t = self.data['novatel']['t_ros'] - self.data['xsens']['t_ros']
        # plt.plot(self.data['novatel']['t_ros'], 'r', label = 'novatel t')
        plt.plot(self.data['novatel']['t_ros'][1:] - self.data['novatel']
                 ['t_ros'][:-1], 'g', label='novatel delta t')
        # plt.plot(self.data['xsens']['t_ros'], 'b', label = 'xsens t')
        plt.legend(loc='upper left')

        plt.subplot(312)
        delta_t = self.data['raw']['t_gps'] - self.data['raw']['t_ros']
        plt.plot(delta_t, 'r', label='raw: t_gps - t_ros')
        plt.legend(loc='upper left')
        plt.xlabel('Stamp Sequence')

        plt.subplot(313)
        delta_t = self.data['raw']['t_gps'][:-1] - \
            self.data['raw']['t_gps'][1:]
        plt.plot(delta_t, 'r', label='raw t_gps delta')
        plt.legend(loc='upper left')
        plt.xlabel('Stamp Sequence')

        plt.show()

        # orientation
        plt.subplot(611)
        plt.plot(self.data['raw']['pitch'], 'r', linewidth=3.0, label='novatel pitch')
        plt.plot(self.data['xsens']['pitch'], 'b', linewidth=3.0, label='xsens pitch')
        plt.legend(loc='upper left')
        plt.xlabel('Stamp Sequence')
        plt.ylabel('Pitch (deg)')
        # plt.title('Xsens Pitch')

        plt.subplot(612)
        delta = self.data['novatel']['pitch'] - self.data['xsens']['pitch']
        self.plot_content(delta, 'pitch accuracy')
        plt.xlabel('Stamp Sequence')
        plt.ylabel('Pitch (deg)')
        # plt.title('Xsens Pitch Accuracy')

        plt.subplot(613)
        plt.plot(self.data['novatel']['roll'], 'r', linewidth=3.0, label='novatel roll')
        plt.plot(self.data['xsens']['roll'], 'b', linewidth=3.0, label='xsens roll')
        plt.legend(loc='upper left')
        plt.xlabel('Stamp Sequence')
        plt.ylabel('Roll (deg)')
        # plt.title('Xsens Roll')

        plt.subplot(614)
        delta = self.data['novatel']['roll'] - self.data['xsens']['roll']
        self.plot_content(delta, 'roll accuracy')
        plt.xlabel('Stamp Sequence')
        plt.ylabel('Roll (deg)')
        # plt.title('Xsens Roll Accuracy')

        plt.subplot(615)
        plt.plot(self.data['raw']['yaw'], 'r', linewidth=3.0, label='novatel yaw')
        plt.plot(self.data['xsens']['yaw'], 'b', linewidth=3.0, label='xsens yaw')
        plt.legend(loc='upper left')
        plt.xlabel('Stamp Sequence')
        plt.ylabel('Yaw (deg)')
        # plt.title('Xsens Yaw')

        plt.subplot(616)
        delta = self.data['novatel']['yaw'] - self.data['xsens']['yaw']
        # delta[abs(delta) > 10] = 0
        self.plot_content(delta, 'yaw accuracy')
        plt.xlabel('Stamp Sequence')
        plt.ylabel('Yaw (deg)')
        # plt.title('Xsens Yaw Accuracy')

        plt.show()

        plt.plot(self.data['novatel']['x'], self.data['novatel']
                 ['y'], 'r', linewidth=3.0, label='novatel trajectory')
        plt.plot(self.data['xsens']['x'], self.data['xsens']
                 ['y'], 'b', linewidth=3.0, label='xsens trajectory')
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.title('Trajectory')
        plt.legend(loc='upper left')

        plt.show()

        # position
        plt.subplot(411)
        delta = self.data['novatel']['x'] - self.data['xsens']['x']
        self.plot_content(delta, 'x accuracy')
        plt.title('Xsens Position accuracy on East direction ')
        plt.ylabel('East (m)')
        plt.xlabel('Stamp Sequence')

        plt.subplot(412)
        delta = self.data['novatel']['y'] - self.data['xsens']['y']
        self.plot_content(delta, 'y accuracy')
        plt.ylabel('North (m)')
        plt.xlabel('Stamp Sequence')

        plt.title('Xsens Position accuracy on North direction ')

        plt.subplot(413)
        plt.plot(self.data['novatel']['vel'], 'r', linewidth=3.0, label='novatel velocity')
        plt.plot(self.data['xsens']['vel'], 'b', linewidth=3.0, label='xsens velocity')
        plt.legend(loc='upper left')
        plt.title('Xsens Velocity')
        plt.ylabel('Velocity (m/s)')
        plt.xlabel('Stamp Sequence')

        plt.subplot(414)
        delta = self.data['novatel']['vel'] - self.data['xsens']['vel']
        self.plot_content(delta, 'velocity accuracy')
        plt.legend(loc='upper left')
        plt.title('Xsens Velocity Accuracy')
        plt.ylabel('Velocity (m/s)')
        plt.xlabel('Stamp Sequence')

        plt.show()

    def plot_content(self, delta, _label):
        plt.plot(delta, 'r', linewidth=3.0, label=_label)
        plt.legend(loc='upper left')

    def run(self):
        self.readbag()
        self.sync()
        self.analyze()
        self.plot()


if __name__ == '__main__':
    # bag = sys.argv[1]
    ori = XsensAnalysis()
    ori.run()
