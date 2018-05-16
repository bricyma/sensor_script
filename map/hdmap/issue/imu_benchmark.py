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


class IMUAnalyzer:
    def __init__(self, info, flag):
        parser = DataParser(info)
        self.file_name = info['bag_name']
        self.ts_begin = info['ts_begin']
        self.ts_end = info['ts_end']

        # get bestpos, insspd, inspvax from dataset

        #  get ros message data
        try:
            corrimu_data1, bestpos_data1, spd_data1, ins_data1, corrimu_data2, bestpos_data2, spd_data2, ins_data2 = parser.parse_dual()
            imu1 = {'corrimu': corrimu_data1, 'bestpos': bestpos_data1, 'ins': ins_data1, 'spd': spd_data1}
            imu2 = {'corrimu': corrimu_data2, 'bestpos': bestpos_data2, 'ins': ins_data2, 'spd': spd_data2}    
        except:
            corrimu_data1, bestpos_data1, spd_data1, ins_data1 = parser.parse_all()
            imu1 = {'corrimu': corrimu_data1, 'bestpos': bestpos_data1, 'ins': ins_data1, 'spd': spd_data1}
            imu2 = imu1

        self.data1 = self.data_warp(imu1)
        self.data2 = self.data_warp(imu2)
        self.imu_pos_offset()
        self.analysis()
    
    def analysis(self):
        items = ['x_acc', 'y_acc', 'z_acc', 'pitch_rate', 'roll_rate', 'yaw_rate']
        print 'inner imu: '
        for item in items:
            print 'std of ' + item, format(np.std(self.data1['corrimu'][item]), '.3f')
        print 'mean of x offset: ', format(np.mean(self.data1['offset']['x']), '.3f')
        print 'mean of y offset: ', format(np.mean(self.data1['offset']['y']), '.3f')
        print 'std of pitch', format(np.std(self.data1['ins']['pitch']), '.3f')
        print 'std of roll', format(np.std(self.data1['ins']['roll']), '.3f')
        print 'outer imu: '
        for item in items:
            print 'std of ' + item, format(np.std(self.data2['corrimu'][item]), '.3f')
        print 'mean of x offset: ', format(np.mean(self.data2['offset']['x']), '.3f')
        print 'mean of y offset: ', format(np.mean(self.data2['offset']['y']), '.3f')

        print 'std of pitch', format(np.std(self.data2['ins']['pitch']), '.3f')
        print 'std of roll', format(np.std(self.data2['ins']['roll']), '.3f')

    def data_warp(self, imu):
        offset = {'x': [], 'y': []}
        corrimu = {'t': [], 'x_acc': [], 'y_acc': [], 'z_acc': [],
                   'pitch_rate': [], 'roll_rate': [], 'yaw_rate': []}
        bestpos = {'x': [], 'y': [], 't': []}
        ins = {'pitch': [], 'roll': [], 'x': [], 'y': [], 'yaw': [], 't': [], 'lat_std': [], 'lon_std': [], 'status': []}
        spd = {'yaw': [], 't': []}
        data = {'corrimu': corrimu, 'bestpos': bestpos, 'ins': ins, 'spd': spd, 'offset': offset}
        # initilize base point, the middle point between start and end point
        transformer = gps_transformer()
        base_lat, base_lon = 0.5  * (imu['ins'][0].latitude + imu['ins'][-1].latitude), 0.5 * (imu['ins'][0].longitude + imu['ins'][-1].longitude)
        data_rate = 125 # IMU-IGM-S1
        for msg in imu['corrimu']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            data['corrimu']['t'].append(pc_time)
            data['corrimu']['x_acc'].append(msg.x_accel * data_rate)
            data['corrimu']['y_acc'].append(msg.y_accel * data_rate)
            data['corrimu']['z_acc'].append(msg.z_accel * data_rate)
            data['corrimu']['pitch_rate'].append(msg.pitch_rate * data_rate)
            data['corrimu']['roll_rate'].append(msg.roll_rate * data_rate)
            data['corrimu']['yaw_rate'].append(msg.yaw_rate * data_rate)

        for msg in imu['spd']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            data['spd']['yaw'].append(msg.track_ground)
            data['spd']['t'].append(pc_time)
        for msg in imu['bestpos']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            x, y = transformer.llh2enu_2(
                msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)
            data['bestpos']['x'].append(x)
            data['bestpos']['y'].append(y)
            data['bestpos']['t'].append(pc_time)
        for msg in imu['ins']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            x, y = transformer.llh2enu_2(
                msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)
            data['ins']['x'].append(x)
            data['ins']['y'].append(y)
            data['ins']['yaw'].append(msg.azimuth)
            data['ins']['roll'].append(msg.roll)
            data['ins']['pitch'].append(msg.pitch)
            # self.data['ins']['t'].append(msg.header.gps_week_seconds)
            data['ins']['t'].append(pc_time)
            data['ins']['lat_std'].append(msg.latitude_std)
            data['ins']['lon_std'].append(msg.longitude_std)
            data['ins']['status'].append(msg.position_type)
        # list to numpy array
        for topic in data:
            for item in data[topic]:
                data[topic][item] = np.array(data[topic][item])
        x_offset, y_offset = self.calculate_offset(data)
        data['offset']['x'], data['offset']['y'] = x_offset, y_offset

        return data

    # leverarm check
    def calculate_offset(self, data):
        x1, y1 = data['bestpos']['x'], data['bestpos']['y']
        x2, y2 = data['ins']['x'], data['ins']['y']
        abs_offset = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        k = np.tan(np.deg2rad(90 - data['ins']['yaw']))
        A = k
        B = -1
        C1 = -k * x1 + y1
        C2 = -k * x2 + y2
        # calculate the x offset
        x_offset = abs(C1 - C2) / np.sqrt(A ** 2 + B ** 2)
        x_offset[x_offset > 2] = 0
        # filter
        y_offset = np.sqrt(abs_offset**2 - x_offset ** 2)
        y_mean = np.mean(y_offset)
        y_offset[y_offset > 4] = y_mean
        y_offset[y_offset < 2] = y_mean

        return x_offset, y_offset

    # dual imu pos check
    def imu_pos_offset(self):
        x1, y1 = self.data1['ins']['x'], self.data1['ins']['y']
        x2, y2 = self.data2['ins']['x'], self.data2['ins']['y']
        abs_offset = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        k = np.tan(np.deg2rad(90 - self.data1['ins']['yaw']))
        A = k
        B = -1
        C1 = -k * x1 + y1
        C2 = -k * x2 + y2
        # calculate the x offset
        x_offset = abs(C1 - C2) / np.sqrt(A ** 2 + B ** 2)
        x_offset[x_offset > 2] = 0
        # filter
        y_offset = np.sqrt(abs_offset**2 - x_offset ** 2)
        y_mean = np.mean(y_offset)
            
        # plt.subplot(211)
        # plt.plot(abs_offset, 'r', label='x offset')
        # plt.legend(loc='upper left')
        # plt.subplot(212)
        # plt.plot(y_offset, 'r', label='y offset')
        # plt.legend(loc='upper left')
        # plt.show()


    def plot_timestamp(self, t_spd_diff, t_ins_diff):
        plt.plot(t_ins_diff, 'r', label='inspvax timestamp space')
        plt.plot(t_spd_diff, 'b', label='insspd timestamp space')
        plt.legend(loc='upper left')
        plt.show()


if __name__ == '__main__':
    # bag_name = '2018-04-09-16-08-16'
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
    # bag_name = '2018-05-10-14-45-41'
    # bag_name = '2018-05-10-17-16-22'
    bag_name = '2018-05-10-18-21-11'
    ts_begin = '0:59'
    ts_end = '100:01'
    info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
    analyzer = IMUAnalyzer(info, 1)

 