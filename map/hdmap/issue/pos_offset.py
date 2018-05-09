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
        # get data from dataset
        rtk_data, ins_data = parser.parse_rtkpos()

        self.rtk_data = {'x': [], 'y': [], 't': []}
        self.ins_data = {'x': [], 'y': [], 'yaw': [], 't': []}

        # warp localization pose data
        self.data_warp(rtk_data, ins_data)
        # self.bag_warp()
        self.calculate_offset()
        self.plot()

    def bag_warp(self):
        transformer = gps_transformer()
        ins_topic_name = '/novatel_data/inspvax'
        rtk_topic_name = '/novatel_data/rtkpos'
        bagname = sys.argv[1]
        bag = rosbag.Bag(bagname)
        for topic, msg, t in bag.read_messages(topics=[ins_topic_name]):
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = transformer.llh2enu_1(
                msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)
                self.ins_data['x'].append(x)
                self.ins_data['y'].append(y)
                self.ins_data['yaw'].append(msg.azimuth)
                self.ins_data['t'].append(msg.header.gps_week_seconds)
        for topic, msg, t in bag.read_messages(topics=[rtk_topic_name]):
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = transformer.llh2enu_1(
                msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)
                self.rtk_data['x'].append(x)
                self.rtk_data['y'].append(y)
                self.rtk_data['t'].append(msg.header.gps_week_seconds)
        
        size = min(len(self.rtk_data['x']), len(self.ins_data['x']))

        for k in self.rtk_data:
            self.rtk_data[k] = np.array(self.rtk_data[k][0:size])
        for k in self.ins_data:
            self.ins_data[k] = np.array(self.ins_data[k][0:size])

        print self.ins_data['t'][-1], self.rtk_data['t'][-1]
        print len(self.ins_data['x']), len(self.rtk_data['x'])


    def data_warp(self, rtk_data, ins_data):
        transformer = gps_transformer()
        base_lat, base_lon = rtk_data[0].latitude, rtk_data[0].longitude

        for msg in rtk_data:
            x, y = transformer.llh2enu_1(
            msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)            
            self.rtk_data['x'].append(x)
            self.rtk_data['y'].append(y)
            self.rtk_data['t'].append(msg.header.gps_week_seconds)
        for msg in ins_data:
            x, y = transformer.llh2enu_1(
            msg.latitude, msg.longitude, 0, base_lat, base_lon, 0)
            self.ins_data['x'].append(x)
            self.ins_data['y'].append(y)
            self.ins_data['yaw'].append(msg.azimuth)
            self.ins_data['t'].append(msg.header.gps_week_seconds)

        for k in self.rtk_data:
            self.rtk_data[k] = np.array(self.rtk_data[k])
        for k in self.ins_data:
            self.ins_data[k] = np.array(self.ins_data[k])
        print self.ins_data['t'][-1], self.rtk_data['t'][-1]

    def calculate_offset(self):
        x1, y1 = self.rtk_data['x'], self.rtk_data['y']
        x2, y2 = self.ins_data['x'], self.ins_data['y']

        abs_offset = np.sqrt((x1-x2)**2 + (y1-y2)**2)
        k = np.tan(np.deg2rad(90 - self.ins_data['yaw'])) 
        A = k
        B = -1
        C1 = -k * x1 + y1
        C2 = -k * x2 + y2
        self.offset = abs(C1-C2)/np.sqrt(A ** 2 + B ** 2)
        self.offset[self.offset > 2] = 0

        self.y_offset = np.sqrt(abs_offset**2 - self.offset ** 2)
        print 'mean of x offset: ', np.mean(self.offset)
        print 'mean of y offset: ', np.mean(self.y_offset)
        
        print 'std of offset: ', np.std(self.offset)

    def plot(self):
        plt.subplot(211)
        plt.plot(self.rtk_data['x'], self.rtk_data['y'], 'r', label='rtkpos')
        plt.plot(self.ins_data['x'], self.ins_data['y'], 'b', label='inspvax')
        plt.legend(loc='upper left')
        plt.subplot(212)
        plt.plot(self.y_offset, 'r', label = 'offset')
        plt.legend(loc='upper left')
        plt.show()



if __name__ == '__main__':
    # bag_name = '2018-03-24-17-52-31' # no error_info
    # bag_name = '2018-03-25-16-35-56'
    # bag_name = '2018-03-25-18-20-44'
    # bag_name = '2018-03-25-19-19-41'
    # bag_name = '2018-03-27-13-46-05'
    # bag_name = '2018-03-27-16-03-26'
    # bag_name = '2018-03-27-21-40-24'
    # bag_name = '2018-03-27-22-47-57'
    # bag_name = '2018-03-28-16-55-36'
    # bag_name = '2018-03-28-18-18-45'
    # bag_name = '2018-03-29-14-28-56'
    # bag_name = '2018-03-29-15-51-57'
    # bag_name = '2018-03-29-16-52-35'
    # bag_name = '2018-03-30-11-29-02'
    # bag_name = '2018-03-30-12-20-17'
    # bag_name = '2018-03-30-14-32-54'
    # bag_name = '2018-03-30-16-42-42'
    # bag_name = '2018-04-02-15-01-12' # novatel failed at 09:55~11:00
    # bag_name = '2018-04-02-17-21-10'
    # bag_name = '2018-03-24-17-52-31'
    # bag_name = '2018-03-29-14-08-55'
    # bag_name = '2018-04-03-14-50-39'
    # bag_name = '2018-04-04-14-30-09'
    # bag_name = '2018-04-04-16-46-39'
    # bag_name = '2018-04-06-11-39-06'
    # bag_name = '2018-04-06-13-39-28'
    # bag_name = '2018-04-09-14-40-14'
    # bag_name = '2018-04-09-16-08-16'
    # bag_name = '2018-05-07-17-15-24'
    # bag_name = '2018-05-03-15-31-35'
    # bag_name = '2018-04-11-14-38-05'

    # B2 outer imu
    # bag_name = '2018-04-30-16-49-43'
    # bag_name = '2018-04-27-15-16-26'
    # bag_name = '2018-04-19-14-57-50'
    # bag_name = '2018-04-17-16-31-11'
    bag_name = '2018-04-16-18-53-27'
    # bag_name = '2018-05-07-17-15-24'
    # bag_name = '2018-05-04-14-42-06'
    # bag_name = '2018-04-30-16-49-43'
    # MKZ test
    # bag_name = '2018-02-28-14-53-01'
    # bag_name = '2018-02-28-18-36-14'
    # bag_name = '2018-03-23-14-42-29'
    # bag_name = '2018-04-02-14-18-15'
    # bag_name = '2018-04-02-15-00-08'
    # bag_name = '2018-04-02-16-29-15'
    # bag_name = '2018-03-18-12-56-25'
    # bag_name = '2018-03-16-11-13-56'
    ts_begin = '0:59'
    ts_end = '121:01'
    info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
    analyzer = ErrorAnalyzer(info)
