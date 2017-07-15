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
            if msg.header.gps_week_seconds % 500 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.ppppos_x.append(x)
                self.ppppos_y.append(y)
                print 'ppp:'
                print msg.header.gps_week_seconds, x, y, msg.latitude_std

        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/rtkpos']):
            if msg.header.gps_week_seconds % 500 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.rtkpos_x.append(x)
                self.rtkpos_y.append(y)
                print 'rtk:'
                print msg.header.gps_week_seconds, x, y

        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/bestpos']):
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.bestpos_x.append(x)
                self.bestpos_y.append(y)
                self.bestpos_time.append(msg.header.gps_week_seconds)

        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            if msg.header.gps_week_seconds % 100 == 0:
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.inspos_x.append(x)
                self.inspos_y.append(y)
                self.inspos_time.append(msg.header.gps_week_seconds)
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



if __name__ == '__main__':
    pos = PosType()
    pos.readbag()

# PPP # 531966500 -305941.761132 19722.8009507
# RTK # 531966500 -305940.486607 19722.5018174
#                         0.7            0.3

# ppp # 531968000 -305941.761463 19722.8028991
# rtk # 531968000 -305940.48757 19722.5016303
#                         0.7         0.3


# 0.8m