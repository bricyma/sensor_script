# x, y looks really weird
import rosbag
import math
import matplotlib.pyplot as plt
import sys
import numpy as np
from llh2enu.llh2enu_gps_transformer import *

EARTH_RADIUS = 6378137.0


class PosType:
    def __init__(self):
        bag_path = '../../rosbag/'
        self.bagname = []
        # assuming there are two bags for comparision
        self.bagname.append(bag_path + sys.argv[1])
        self.bagname.append(bag_path + sys.argv[2])

        self.transform = gps_transformer()

        self.data = [{}, {}]
        for i in range(2):
            self.data[i]['rtkpos_x'] = []
            self.data[i]['rtkpos_y'] = []
            self.data[i]['bestpos_x'] = []
            self.data[i]['bestpos_y'] = []
            self.data[i]['ins_x'] = []
            self.data[i]['ins_y'] = []
        # SD
        # self.baseLat = self.DEG2RAD(32.694052)
        # self.baseLon = self.DEG2RAD(-113.958389)
        # caofeidian
        self.baseLat = self.DEG2RAD(39.5096245928)
        self.baseLon = self.DEG2RAD(118.110978652)

        self.limit = 5000000

    def readbag(self, index):
        bag = rosbag.Bag(self.bagname[index])
        count = 0
        cur = 0
        for topic, msg, t in bag.read_messages(topics=['/novatel_data/rtkpos']):
            count += 1
            if count > self.limit:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data[index]['rtkpos_x'].append(x)
                self.data[index]['rtkpos_y'].append(y)
        cur = 0
        count = 0
        for topic, msg, t in bag.read_messages(topics=['/novatel_data/bestpos']):
            count += 1
            if count > self.limit:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.transform.llh2enu_5(msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data[index]['bestpos_x'].append(x)
                self.data[index]['bestpos_y'].append(y)
        cur = 0
        count = 0
        for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
            count += 1
            if count > self.limit * 2.5:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.transform.llh2enu_5(
                    msg.latitude, msg.longitude, 0, self.baseLat, self.baseLon, 0)
                self.data[index]['ins_x'].append(x)
                self.data[index]['ins_y'].append(y)
                cur += 1
        bag.close()

    def DEG2RAD(self, x):
        return x / (180 / math.pi)

    def RAD2DEG(self, x):
        return x * (180 / math.pi)

    def plot(self):
        # plt.subplot(211)
        # plt.plot(self.bestpos_x, self.bestpos_y, 'r', marker='o', label='bestpos')
        plt.plot(self.data[0]['ins_x'], self.data[0]
                 ['ins_y'], 'r', marker='o', label='inspos0')
        plt.plot(self.data[1]['ins_x'], self.data[1]
                 ['ins_y'], 'b', marker='o', label='inspos1')
        # plt.plot(self.ppppos_x, self.ppppos_y, 'g', label='ppppos')
        # plt.plot(self.rtkpos_x, self.rtkpos_y, 'y', label='rtkpos')

        # plt.plot(self.bestpos_x[0], self.bestpos_y[0], color='green', linewidth=100, label='start')
        plt.legend(loc='upper left')
        # plt.subplot(212)
        # plt.plot([np.mean(self.distance)] * len(self.distance), 'g', linestyle='-', linewidth=2, label='average')

        plt.show()


if __name__ == '__main__':
    pos = PosType()
    pos.readbag(0)
    pos.readbag(1)
    pos.plot()
