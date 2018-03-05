# x, y looks really weird
import rosbag
import math
import matplotlib.pyplot as plt
import sys
import numpy as np

EARTH_RADIUS = 6378137.0


class PosType:
    def __init__(self):
        bag_path = '../../rosbag/20180206/lmd/'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.inspos = {}
        self.namespace = ['novatel6', 'novatel7']
        for ns in self.namespace:
            self.inspos[ns] = {}
            self.inspos[ns]['x'] = []
            self.inspos[ns]['y'] = []

        # caofeidian
        self.baseLat = self.DEG2RAD(39.5096245928)
        self.baseLon = self.DEG2RAD(118.110978652)

    def readbag(self):
        for ns in self.namespace:
            for topic, msg, t in self.bag.read_messages(topics=['/' + ns + '/novatel_data/inspvax']):
                if msg.header.gps_week_seconds % 100 == 0:
                    x, y = self.latlon2xy(msg.latitude, msg.longitude)
                    self.inspos[ns]['x'].append(x)
                    self.inspos[ns]['y'].append(y)
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
        i = 0
        for ns in self.namespace:
            if i==0:
                plt.plot(self.inspos[ns]['x'], self.inspos[ns]['y'], 'bo', label=ns)
                i = 1
            else:
                plt.plot(self.inspos[ns]['x'], self.inspos[ns]['y'], 'ro', label=ns)
        plt.legend(loc='upper left')
        plt.title(sys.argv[1])
        plt.show()


if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
    pos.plot()
