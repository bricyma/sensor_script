# #!/usr/bin/env python

# analyze the relationship between postion deviation and yaw offset
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
from llh2enu.llh2enu_gps_transformer import *
import sys
import rosbag

base_lat = 32.75707  # center between tucson and phoenix
base_lon = -111.55757


class ErrorAnalyzer:
    def __init__(self, info):
        parser = DataParser(info)
        self.file_name = info['bag_name']
        self.ts_begin = info['ts_begin']
        self.ts_end = info['ts_end']

        # get bestpos, insspd, inspvax from dataset
        corrimu_data, bestpos_data, spd_data, ins_data = parser.parse_all()
        self.ori_data = {'corrimu': corrimu_data,
                         'bestpos': bestpos_data, 'ins': ins_data, 'spd': spd_data}
        corrimu = {'t': [], 'x_acc': [], 'y_acc': [], 'z_acc': [],
                   'pitch_rate': [], 'roll_rate': [], 'yaw_rate': []}
        bestpos = {'x': [], 'y': [], 't': []}
        ins = {'x': [], 'y': [], 'yaw': [], 't': []}
        spd = {'yaw': [], 't': []}
        self.data = {'corrimu': corrimu,
                     'bestpos': bestpos, 'ins': ins, 'spd': spd}
        self.data_warp()

        x_offset, y_offset = self.calculate_offset()
        self.plot_offset(x_offset, y_offset)

        yaw_diff = self.yaw_check()
        self.plot_yaw(yaw_diff)

        t_spd_diff, t_ins_diff = self.timestamp_check()
        self.plot_timestamp(t_spd_diff, t_ins_diff)

        self.plot_imu()

    def data_warp(self):
        # initilize base point
        transformer = gps_transformer()
        base_lat, base_lon = self.ori_data['ins'][0].latitude, self.ori_data['ins'][0].longitude

        for msg in self.ori_data['corrimu']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            self.data['corrimu']['t'].append(pc_time)
            self.data['corrimu']['x_acc'].append(msg.x_accel)
            self.data['corrimu']['y_acc'].append(msg.y_accel)
            self.data['corrimu']['z_acc'].append(msg.z_accel)
            self.data['corrimu']['pitch_rate'].append(msg.pitch_rate)
            self.data['corrimu']['roll_rate'].append(msg.roll_rate)
            self.data['corrimu']['yaw_rate'].append(msg.yaw_rate)

        for msg in self.ori_data['spd']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            self.data['spd']['yaw'].append(msg.track_ground)
            self.data['spd']['t'].append(pc_time)
        for msg in self.ori_data['bestpos']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            x, y = transformer.llh2enu_1(
                msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)
            self.data['bestpos']['x'].append(x)
            self.data['bestpos']['y'].append(y)
            self.data['bestpos']['t'].append(pc_time)
        for msg in self.ori_data['ins']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            x, y = transformer.llh2enu_1(
                msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)
            self.data['ins']['x'].append(x)
            self.data['ins']['y'].append(y)
            self.data['ins']['yaw'].append(msg.azimuth)
            # self.data['ins']['t'].append(msg.header.gps_week_seconds)
            self.data['ins']['t'].append(pc_time)

        # list to numpy array
        for topic in self.data:
            for item in self.data[topic]:
                self.data[topic][item] = np.array(self.data[topic][item])

    # leverarm check
    def calculate_offset(self):
        x1, y1 = self.data['bestpos']['x'], self.data['bestpos']['y']
        x2, y2 = self.data['ins']['x'], self.data['ins']['y']
        abs_offset = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        k = np.tan(np.deg2rad(90 - self.data['ins']['yaw']))
        A = k
        B = -1
        C1 = -k * x1 + y1
        C2 = -k * x2 + y2
        # calculate the x offset
        x_offset = abs(C1 - C2) / np.sqrt(A ** 2 + B ** 2)
        x_offset[x_offset > 2] = 0
        # filter
        y_offset = np.sqrt(abs_offset**2 - x_offset ** 2)
        y_offset[y_offset > 5] = 3.5
        y_offset[y_offset < 2.5] = 3.5

        print 'mean of x offset: ', np.mean(x_offset)
        print 'mean of y offset: ', np.mean(y_offset)
        print 'std of x offset: ', np.std(x_offset)
        print 'std of y offset: ', np.std(y_offset)

        return x_offset, y_offset

    # rotation between vehicle and imu body check
    def yaw_check(self):
        yaw_diff = self.data['spd']['yaw'] - self.data['ins']['yaw']
        yaw_diff[abs(yaw_diff) > 2] = 0
        print 'mean of (spd yaw - ins yaw)', np.mean(yaw_diff)
        return yaw_diff

    # check yaw rate
    def yaw_rate_check(self):
         # delta_yaw_rate = self.data['']
        pass

    # check timestamp
    def timestamp_check(self):
        t_spd, t_ins = self.data['spd']['t'], self.data['ins']['t']
        t_spd_diff = t_spd[1:] - t_spd[:-1]
        t_ins_diff = t_ins[1:] - t_ins[:-1]
        return t_spd_diff, t_ins_diff

    # check acceleration, angular rate of CORRIMUDATA
    def plot_imu(self):
        # acceleration
        plt.subplot(311)
        plt.plot(self.data['corrimu']['x_acc'], 'r', label='x_acc')
        plt.legend(loc='upper left')
        plt.subplot(312)
        plt.plot(self.data['corrimu']['y_acc'], 'r', label='y_acc')
        plt.legend(loc='upper left')
        plt.subplot(313)
        plt.plot(self.data['corrimu']['z_acc'], 'r', label='z_acc')
        plt.legend(loc='upper left')
        plt.show()
        # angular rate
        plt.subplot(311)
        plt.plot(self.data['corrimu']['pitch_rate'], 'r', label='pitch_rate')
        plt.legend(loc='upper left')
        plt.subplot(312)
        plt.plot(self.data['corrimu']['roll_rate'], 'r', label='roll_rate')
        plt.legend(loc='upper left')
        plt.subplot(313)
        plt.plot(self.data['corrimu']['yaw_rate'], 'r', label='yaw_rate')
        plt.legend(loc='upper left')
        plt.show()

    def plot_offset(self, x_offset, y_offset):
        plt.subplot(311)
        plt.plot(self.data['bestpos']['x'],
                 self.data['bestpos']['y'], 'r', label='bestpos')
        plt.plot(self.data['ins']['x'], self.data['ins']
                 ['y'], 'b', label='inspvax')
        plt.legend(loc='upper left')
        plt.subplot(312)
        plt.plot(x_offset, 'r', label='x offset')
        plt.legend(loc='upper left')
        plt.subplot(313)
        plt.plot(y_offset, 'r', label='y offset')
        plt.legend(loc='upper left')
        plt.show()

    def plot_yaw(self, yaw_diff):
        plt.subplot(211)
        plt.plot(self.data['ins']['yaw'], 'r', label='inspvax yaw')
        plt.plot(self.data['spd']['yaw'], 'b', label='insspd yaw')
        plt.legend(loc='upper left')
        plt.subplot(212)
        plt.plot(yaw_diff, 'r', label='spd yaw - ins yaw')
        plt.legend(loc='upper left')
        plt.show()

    def plot_timestamp(self, t_spd_diff, t_ins_diff):
        plt.plot(t_ins_diff, 'r', label='inspvax timestamp space')
        plt.plot(t_spd_diff, 'b', label='insspd timestamp space')
        plt.legend(loc='upper left')
        plt.show()


if __name__ == '__main__':
    # B2 outer imu
    # bag_name = '2018-05-04-14-42-06'
    # bag_name = '2018-04-30-16-49-43'
    # bag_name = '2018-05-08-16-48-29'
    # bag_name = '2018-05-08-16-48-29'
    # bag_name = '2018-05-07-17-15-24'
    # bag_name = '2018-05-09-17-14-03'
    # bag_name = '2018-05-10-12-14-54'
    # bag_name = '2018-05-09-18-52-08'
    # bag_name = '2018-05-09-17-25-34'
    bag_name = '2018-05-10-14-45-41'
    
    ts_begin = '0:59'
    ts_end = '100:01'
    info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
    analyzer = ErrorAnalyzer(info)
