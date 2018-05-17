# #!/usr/bin/env python

# analyze the relationship between postion deviation and yaw offset
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
from llh2enu.llh2enu_gps_transformer import *
import sys
import rosbag
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy


class IMUAnalyzer:
    def __init__(self, info, flag):
        parser = DataParser(info)
        self.file_name = info['bag_name']
        self.ts_begin = info['ts_begin']
        self.ts_end = info['ts_end']


        self.base_lat = 32.75707  # center between tucson and phoenix
        self.base_lon = -111.55757

        # get bestpos, insspd, inspvax from dataset

        #  get ros message data
        try:
            corrimu_data1, bestpos_data1, spd_data1, ins_data1, corrimu_data2, bestpos_data2, spd_data2, ins_data2 = parser.parse_dual()
            imu1 = {'corrimu': corrimu_data1, 'bestpos': bestpos_data1,
                    'ins': ins_data1, 'spd': spd_data1}
            imu2 = {'corrimu': corrimu_data2, 'bestpos': bestpos_data2,
                    'ins': ins_data2, 'spd': spd_data2}
        except:
            corrimu_data1, bestpos_data1, spd_data1, ins_data1 = parser.parse_all()
            imu1 = {'corrimu': corrimu_data1, 'bestpos': bestpos_data1,
                    'ins': ins_data1, 'spd': spd_data1}
            imu2 = imu1

        self.data1 = self.data_warp(imu1)
        self.data2 = self.data_warp(imu2)
        self.analysis()

    def analysis(self):
        items = ['x_acc', 'y_acc', 'z_acc',
                 'pitch_rate', 'roll_rate', 'yaw_rate']
        print 'inner imu: '
        for item in items:
            print 'std of ' + item, format(np.std(self.data1['corrimu'][item]), '.3f')
        print 'mean of x offset: ', format(np.mean(self.data1['offset']['x']), '.4f')
        print 'mean of y offset: ', format(np.mean(self.data1['offset']['y']), '.3f')
        print 'std of pitch', format(np.std(self.data1['ins']['pitch']), '.3f')
        print 'std of roll', format(np.std(self.data1['ins']['roll']), '.3f')
        print 'mean of pos x offset: ', format(np.mean(self.data1['pos']['x']), '.4f')
        print 'mean of pos y offset: ', format(np.mean(self.data1['pos']['y']), '.3f')

        print 'outer imu: '
        for item in items:
            print 'std of ' + item, format(np.std(self.data2['corrimu'][item]), '.3f')
        print 'mean of x offset: ', format(np.mean(self.data2['offset']['x']), '.3f')
        print 'mean of y offset: ', format(np.mean(self.data2['offset']['y']), '.3f')
        print 'std of pitch', format(np.std(self.data2['ins']['pitch']), '.3f')
        print 'std of roll', format(np.std(self.data2['ins']['roll']), '.3f')

    def data_warp(self, imu):
        pos =  {'x': [], 'y': []}
        offset = {'x': [], 'y': []}
        corrimu = {'t': [], 'x_acc': [], 'y_acc': [], 'z_acc': [],
                   'pitch_rate': [], 'roll_rate': [], 'yaw_rate': []}
        bestpos = {'x': [], 'y': [], 't': [],
                   'diff_age': [], 'status': [], 'sol_age': []}
        ins = {'pitch': [], 'roll': [], 'x': [], 'y': [], 'yaw': [],
               't': [], 'lat_std': [], 'lon_std': [], 'status': []}
        spd = {'yaw': [], 't': []}
        data = {'corrimu': corrimu, 'bestpos': bestpos,
                'ins': ins, 'spd': spd, 'offset': offset, 'pos': pos}
        # initilize base point, the middle point between start and end point
        transformer = gps_transformer()
        # self.base_lat, self.base_lon = 0.5 * (imu['ins'][0].latitude + imu['ins'][-1].latitude), 0.5 * (
            # imu['ins'][0].longitude + imu['ins'][-1].longitude)
        data_rate = 125  # IMU-IGM-S1
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
            x, y = transformer.llh2enu_5(
                msg.latitude, msg.longitude, 0, self.base_lat, self.base_lon, 0)
            data['bestpos']['x'].append(x)
            data['bestpos']['y'].append(y)
            data['bestpos']['t'].append(pc_time)
            data['bestpos']['diff_age'].append(msg.diff_age)
            data['bestpos']['sol_age'].append(msg.sol_age)
            data['bestpos']['status'].append(msg.position_type)
        for msg in imu['ins']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            x, y = transformer.llh2enu_5(
                msg.latitude, msg.longitude, 0, self.base_lat, self.base_lon, 0)
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

        # leverarm check
        data['offset']['x'], data['offset']['y'] = x_offset, y_offset

        # position offset check
        data['pos']['x'], data['pos']['y'] = self.check_pos(data)

        return data

    # leverarm check
    def calculate_offset(self, data):
        x1, y1 = data['bestpos']['x'], data['bestpos']['y']
        x2, y2 = data['ins']['x'], data['ins']['y']

        x_offset, y_offset = self.get_distance(
            x1, y1, x2, y2, data['ins']['yaw'])

        x_offset[x_offset > 2] = 0
        y_mean = np.mean(y_offset)
        # filter
        y_offset[y_offset > 4] = y_mean
        y_offset[y_offset < 2] = y_mean


        return x_offset, y_offset

    # distance between INSPVAX and HDmap lane center
    def check_pos(self, data):
        # map_file = "I-10_Tucson2Phoenix_20180412.hdmap"
        map_file = "I-10_Phoenix2Tucson_20180412.hdmap"
        with open(map_file, 'rb') as f:
            submap = f.read()
        hdmap = TSMap(submap)
        x1, y1 = data['bestpos']['x'], data['bestpos']['y']
        x2, y2 = [], []
        for i in range(len(x1)):
            x, y = x1[i], y1[i]
            p = Point3d(x, y)
            try:
                ref_p = hdmap.get_ref_pt(p)
            except:
                ref_p = p
            x2.append(ref_p.x)
            y2.append(ref_p.y)
        x2, y2 = np.array(x2), np.array(y2)
        x_offset, y_offset = self.get_distance(
            x1, y1, x2, y2, data['ins']['yaw'])
        return x_offset, y_offset

    # input: x1, y1, x2, y2, yaw
    # output: x offset, y offset
    def get_distance(self, x1, y1, x2, y2, yaw):
        abs_offset = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        k = np.tan(np.deg2rad(90 - yaw))
        A = k
        B = -1
        C1 = -k * x1 + y1
        C2 = -k * x2 + y2
        # calculate the x offset
        x_offset = abs(C1 - C2) / np.sqrt(A ** 2 + B ** 2)
        y_offset = np.sqrt(abs_offset**2 - x_offset ** 2)
        return x_offset, y_offset

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
    # bag_name = '2018-05-10-18-21-11'
    # bag_name = '2018-05-16-15-43-19'
    bag_name = '2018-05-16-15-43-19'

    ts_begin = '0:59'
    ts_end = '100:01'
    info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
    analyzer = IMUAnalyzer(info, 1)
