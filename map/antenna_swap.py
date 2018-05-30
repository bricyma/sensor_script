# x, y looks really weird
import rosbag
import math
import matplotlib.pyplot as plt
import sys
import numpy as np

EARTH_RADIUS = 6378137.0


class PosType:
    def __init__(self):
        bagname1 = sys.argv[1]
        bagname2 = sys.argv[2]
        self.bag1 = rosbag.Bag(bagname1)
        self.bag2 = rosbag.Bag(bagname2)

        self.rtkpos_x = []
        self.rtkpos_y = []

        self.bestpos_x = []
        self.bestpos_y = []
        self.inspos_x = []
        self.inspos_y = []
        self.inspos_yaw = []
        self.bestpos_time = []
        self.inspos_time = []
        self.distance = []
        self.baseLat = self.DEG2RAD(32.0922419)
        self.baseLon = self.DEG2RAD(-110.7894856)
        self.limit = 5000000
        self.pos1 = {'x':0, 'y':0}
        self.pos2 = {'x':0, 'y': 0}
        self.bag1_len = 0

    def readbag(self, bag_name):
        count = 0
        cur = 0
        count = 0
        for topic, msg, t in bag_name.read_messages(topics=['/novatel_data/rtkpos']):
            count += 1
            if count > self.limit:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.rtkpos_x.append(x)
                self.rtkpos_y.append(y)
        cur = 0
        count = 0
        for topic, msg, t in bag_name.read_messages(topics=['/novatel_data/bestpos']):
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
        for topic, msg, t in bag_name.read_messages(topics=['/novatel_data/inspvax']):
            count += 1
            if count > self.limit * 2.5:
                break
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.inspos_x.append(x)
                self.inspos_y.append(y)
                self.inspos_yaw.append(msg.azimuth)
                self.inspos_time.append(msg.header.gps_week_seconds)
                cur += 1

        if self.pos1['x'] == 0:
            self.pos1['x'], self.pos1['y'] = np.mean(self.bestpos_x), np.mean(self.bestpos_y)
            self.bag1_len = len(self.bestpos_x)
        else:
            bag2_len = len(self.bestpos_x) - self.bag1_len
            self.pos2['x'] = (sum(self.bestpos_x) - self.pos1['x'] * self.bag1_len) / bag2_len
            self.pos2['y'] = (sum(self.bestpos_y) - self.pos1['y'] * self.bag1_len) / bag2_len

            print 'dual antenna heading: ', 90 + np.rad2deg(np.arctan2(self.pos1['x'] - self.pos2['x'], self.pos1['y'] - self.pos2['y']))
            print 'inspvax yaw: ', np.mean(self.inspos_yaw)
            print 'leverarm: ', np.sqrt((self.pos1['x'] - self.pos2['x'])**2 + (self.pos1['y'] - self.pos2['y'])**2)
        bag_name.close()

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
        y = math.log(math.tan(math.asin(zz) / 2 + math.pi / 4)) * EARTH_RADIUS
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
        plt.plot(self.bestpos_x, self.bestpos_y, 'r', marker='o', label='bestpos')
        plt.plot(self.inspos_x, self.inspos_y, 'b', marker='o', label='inspos')
        # plt.plot(self.rtkpos_x, self.rtkpos_y, 'y', label='rtkpos')
        plt.legend(loc='upper left')
        plt.show()


if __name__ == '__main__':
    pos = PosType()
    pos.readbag(pos.bag1)
    pos.readbag(pos.bag2)
    pos.plot()
