# x, y looks really weird
import rosbag
import math
import matplotlib.pyplot as plt
import sys
import numpy as np

EARTH_RADIUS = 6378137.0


class PosType:
    def __init__(self):
        bag_path = '../../rosbag/20180206/whd/'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.inspos = {}
        self.namespace = ['novatel6', 'novatel7']
        for ns in self.namespace:
            self.inspos[ns] = {}
            self.inspos[ns]['x'] = []
            self.inspos[ns]['y'] = []
            self.inspos[ns]['yaw'] = []
            self.inspos[ns]['yaw_std'] = []
            self.inspos[ns]['lat_std'] = []
            self.inspos[ns]['pos_type'] = []  # 56: rtk fixed, 55: RTK float, 54: PSDIFF, 53: PSP 19: Propagated
        # caofeidian
        self.baseLat = self.DEG2RAD(38.9927082634)
        self.baseLon = self.DEG2RAD(118.469187136)

    def readbag(self):
        for ns in self.namespace:
            for topic, msg, t in self.bag.read_messages(topics=['/' + ns + '/novatel_data/inspvax']):
                x, y = self.latlon2xy(msg.latitude, msg.longitude)
                self.inspos[ns]['x'].append(x)
                self.inspos[ns]['y'].append(y)
                self.inspos[ns]['yaw'].append(msg.azimuth)
                self.inspos[ns]['yaw_std'].append(msg.azimuth_std)
                self.inspos[ns]['lat_std'].append(msg.latitude_std)
                self.inspos[ns]['pos_type'].append(msg.position_type)
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

    def plot_fig(self, n, num):
        i = 0
        # plot x,y
        if num == 2:
            for ns in self.namespace:
                if i == 0:
                    plt.plot(self.inspos[ns]['x'],
                             self.inspos[ns]['y'], 'bo', label=ns)
                    i = 1
                else:
                    plt.plot(self.inspos[ns]['x'],
                             self.inspos[ns]['y'], 'ro', label=ns)
        else:  # plot yaw, yaw_std, position_type
            for ns in self.namespace:
                if i == 0:
                    plt.plot(self.inspos[ns][n], 'b', label=ns)
                    i = 1
                else:
                    plt.plot(self.inspos[ns][n], 'ro', label=ns)
        plt.legend(loc='upper left')
        plt.title(n + ': ' + sys.argv[1])

    def plot(self):
        plt.subplot(511)
        self.plot_fig('x', 2)
        plt.subplot(512)
        self.plot_fig('lat_std', 1)
        plt.subplot(513)
        self.plot_fig('yaw', 1)
        plt.subplot(514)
        self.plot_fig('yaw_std', 1)
        plt.subplot(515)
        self.plot_fig('pos_type', 1)
        plt.show()

if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
    print 'yaw std: novatel6 vs novatel 7'
    print np.std(pos.inspos['novatel6']['yaw']), ' ', np.std(pos.inspos['novatel7']['yaw'])
    print 'position std: novatel6 vs novatel 7'
    average6_x = np.mean(pos.inspos['novatel6']['x'])
    average6_y = np.mean(pos.inspos['novatel6']['y'])
    average7_x = np.mean(pos.inspos['novatel7']['x'])
    average7_y = np.mean(pos.inspos['novatel7']['y'])
    print 'p6 x_std: ', np.std(pos.inspos['novatel6']['x'] - average6_x)
    print 'p6 y_std: ', np.std(pos.inspos['novatel6']['y'] - average6_y)
    print 'p7_x_std: ', np.std(pos.inspos['novatel7']['x'] - average7_x)
    print 'p7_y_std: ', np.std(pos.inspos['novatel7']['y'] - average7_y)

    pos.plot()
