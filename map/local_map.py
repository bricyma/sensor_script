# x, y looks really weird
import rosbag
import math
import matplotlib.pyplot as plt
import sys
import numpy as np

EARTH_RADIUS = 6378137.0

class PosType:
    def __init__(self):
        bag_path = '../../rosbag/'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.ppppos_x = []
        self.ppppos_y = []

        self.rtkpos_x = []
        self.rtkpos_y = []

        self.bestpos_x = []
        self.bestpos_y = []
        self.inspos_x = []
        self.inspos_y = []
        self.bestpos_time = []
        self.inspos_time = []
        self.distance = []
        self.baseLat = self.DEG2RAD(32.694052)
        self.baseLon = self.DEG2RAD(-113.958389)
        self.limit = 5000000
    def readbag(self):
        count = 0
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/ppppos']):
            count += 1
            if count > self.limit:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.ppppos_x.append(x)
                self.ppppos_y.append(y)
                # self.bestpos_time.append(msg.header.gps_week_seconds)
        cur = 0
        count = 0
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/rtkpos']):
            count += 1
            if count > self.limit:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.rtkpos_x.append(x)
                self.rtkpos_y.append(y)
                # self.bestpos_time.append(msg.header.gps_week_seconds)
        cur = 0
        count = 0
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/bestpos']):
            count += 1
            if count > self.limit:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.bestpos_x.append(x)
                self.bestpos_y.append(y)
                self.bestpos_time.append(msg.header.gps_week_seconds)
        cur = 0
        count = 0
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            # if cur < len(self.bestpos_time) and msg.header.gps_week_seconds == self.bestpos_time[cur]:
            count += 1
            if count > self.limit * 2.5:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.inspos_x.append(x)
                self.inspos_y.append(y)
                self.inspos_time.append(msg.header.gps_week_seconds)
                cur += 1
        # for i in range(0, len(self.bestpos_time)):
            # if np.sqrt((pos.inspos_x[i] - pos.bestpos_x[i]) ** 2 + (pos.inspos_y[i] - pos.inspos_y[i]) ** 2) < 1:
            #     self.distance.append(np.sqrt((pos.inspos_x[i] - pos.bestpos_x[i]) ** 2 + (pos.inspos_y[i] - pos.inspos_y[i]) ** 2))
        self.bag.close()

    def DEG2RAD(self, x):
        return x / (180 / math.pi)

    def RAD2DEG(self, x):
        return x * (180 / math.pi)

    # method 1
    def latlon2xy(self, lat_, lon_):
        lat, lon = self.DEG2RAD(lat_), self.DEG2RAD(lon_)
        xx = math.cos(lat) * math.cos(lon) * math.cos(self.baseLon) * math.cos(self.baseLat) \
             + math.cos(lat) * math.sin(lon) * math.sin(self.baseLon) * math.cos(self.baseLat) \
             + math.sin(lat) * math.sin(self.baseLat)
        yy = -math.cos(lat) * math.cos(lon) * math.sin(self.baseLon) \
             + math.cos(lat) * math.sin(lon) * math.cos(self.baseLon)
        zz = -math.cos(lat) * math.cos(lon) * math.cos(self.baseLon) * math.sin(self.baseLat) \
             - math.cos(lat) * math.sin(lon) * math.sin(self.baseLon) * math.sin(self.baseLat) \
             + math.sin(lat) * math.cos(self.baseLat)
        x = math.atan2(yy, xx) * EARTH_RADIUS
        y = math.log(math.tan(math.asin(zz) / 2 + math.pi/4 )) * EARTH_RADIUS
        return x, y

    # method 2
    def transform(self, lat, lon):
        er = 6378137.0
        lat0 = 0
        s = np.cos(lat0 * math.pi / 180)
        tx = s * er * math.pi * lon / 180.0
        ty = s * er * np.log(np.tan((90.0 + lat) * math.pi / 360.0))
        return tx, ty

    def plot(self):
        plt.subplot(211)
        plt.plot(self.bestpos_x, self.bestpos_y, 'r', marker='o', label='bestpos')
        plt.plot(self.inspos_x, self.inspos_y, 'b', marker='o', label='inspos')
        plt.plot(self.ppppos_x, self.ppppos_y, 'g', label='ppppos')
        plt.plot(self.rtkpos_x, self.rtkpos_y, 'y', label='rtkpos')

        # plt.plot(self.bestpos_x[0], self.bestpos_y[0], color='green', linewidth=100, label='start')
        plt.legend(loc='upper left')
        plt.subplot(212)
        plt.plot(self.distance, 'r')
        plt.plot([np.mean(self.distance)] * len(self.distance), 'g', linestyle='-', linewidth=2, label='average')

        plt.show()

if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
    print 'real distance: ', np.sqrt(0.329**2+0.58**2)
    print np.mean(pos.distance)
    print len(pos.bestpos_x), len(pos.inspos_x)
    pos.plot()
