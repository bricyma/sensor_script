# python orientation.py *.bag
# author: Zhibei Ma
# date: 2017/06/29
# check the offset angle between imu and vehicle's yaw

import matplotlib.pyplot as plt
import math as m
import rosbag
import sys
import numpy as np
import math
from llh2enu.llh2enu_gps_transformer import *

EARTH_RADIUS = 6378137.0


class Orientation:
    def __init__(self, bagname):
        bag_path = ''
        bag = bag_path + bagname
        self.bag = rosbag.Bag(bag)
        self.north_vel = []
        self.east_vel = []
        self.yaw = []
        self.ori_vel = []  # orientation calculated from east_velocity and north_velocity
        self.diff = []   # gps yaw - inspvax
        self.diff2 = []  # bestvel - inspvax
        self.data = {}
        self.baseLat = 0.693143165823  # caofeidian
        self.baseLon = 2.04736661229
        self.transform = gps_transformer()
        self.baseLat = 39.03775082210  # wuhudao
        self.baseLon = 118.43091220755

        # self.baseLat = 31.344678  # shanghai
        # self.baseLon = 121.379186

        # self.baseLat = 39.714178  # beijing
        # self.baseLon = 117.305466
        # self.baseLat = 32.694052  # san diego
        # self.baseLon = -113.958389

        print 'baseLat: ', self.baseLat
        print 'baseLon: ', self.baseLon

    def readbag(self):
        self.data['lat'] = []
        self.data['lon'] = []
        self.data['pitch'] = []
        self.data['roll'] = []
        self.data['yaw'] = []
        self.data['cal_diff'] = []
        self.data['best_yaw'] = []
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/bestvel']):
            self.data['best_yaw'].append(msg.trk_gnd)
        for topic, msg, t in self.bag.read_messages(topics=['/calibrate/diff']):
            if msg.data > 2 or msg.data < -2:
                msg.data = 0
            self.data['cal_diff'].append(msg.data)

        first_flag = False  # choose the start point as the base, if True
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            if first_flag:
                self.baseLat = msg.latitude
                self.baseLon = msg.longitude
                first_flag = False
            self.north_vel.append(msg.north_velocity)
            self.east_vel.append(msg.east_velocity)
            self.data['yaw'].append(msg.azimuth)
            self.data['roll'].append(msg.roll)
            self.data['pitch'].append(msg.pitch)
            self.data['lat'].append(msg.latitude)
            self.data['lon'].append(msg.longitude)
        self.bag.close()

    def gps2orientation(self):
        self.data['gps_yaw'] = []
        for i, lat in enumerate(self.data['lat']):
            if i + 2 > len(self.data['lat']):
                break
            lon = self.data['lon'][i]
            x, y = self.transform.llh2enu_5(
                lat, lon, 0, self.baseLat, self.baseLon, 0)
            lat2 = self.data['lat'][i + 1]
            lon2 = self.data['lon'][i + 1]
            x2, y2 = self.transform.llh2enu_5(
                lat2, lon2, 0, self.baseLat, self.baseLon, 0)

            gps_yaw = self.RAD2DEG(np.arctan2(x2 - x, y2 - y))
            if gps_yaw < 0:
                gps_yaw += 360
            self.data['gps_yaw'].append(gps_yaw)

    def orientation_from_velocity(self):
        for i in range(0, len(self.north_vel)):
            if np.arctan2(self.east_vel[i], self.north_vel[i]) < 0:
                cur_ori = np.arctan2(
                    self.east_vel[i], self.north_vel[i]) * 180 / np.pi + 360
            else:
                cur_ori = np.arctan2(
                    self.east_vel[i], self.north_vel[i]) * 180 / np.pi

            self.ori_vel.append(cur_ori)

    def compare(self):
        yaw1 = np.array(self.data['yaw'])
        yaw2 = np.array(self.data['gps_yaw'])
        yaw_ins = np.array(self.data['yaw'][::5])
        yaw_gps = np.array(self.data['best_yaw'][::2])

        size = len(yaw_ins) if len(yaw_ins) < len(yaw_gps) else len(yaw_gps)
        yaw_ins = yaw_ins[0:size]
        yaw_gps = yaw_gps[0:size]
        self.diff2 = yaw_gps - yaw_ins
        self.diff2[self.diff2 > 0.5] = 0
        self.diff2[self.diff2 < -0.5] = 0

        self.diff = yaw2 - yaw1[:-1]
        self.diff[self.diff > 0.5] = 0
        self.diff[self.diff < -0.5] = 0

        print 'mean diff: ', np.mean(self.diff)
        print 'mean diff2 bestvel: ', np.mean(self.diff2)

    def plot(self):
        plt.subplot(411)
        plt.plot(self.data['pitch'], 'r', label='pitch')
        plt.legend(loc='upper left')
        plt.subplot(412)
        plt.plot(self.data['roll'], 'r', label='roll')
        plt.legend(loc='upper left')
        plt.subplot(413)
        plt.plot(self.data['yaw'], 'rx', label='yaw')
        plt.legend(loc='upper left')
        plt.subplot(414)
        plt.legend(loc='upper left')
        plt.show()

    # latitude longiutde to enu
    def DEG2RAD(self, x):
        return x / (180 / math.pi)

    def RAD2DEG(self, x):
        return x * (180 / math.pi)

    # method 2
    def latlon2xy(self, lat, lon):
        er = 6378137.0
        lat0 = 0
        s = np.cos(self.baseLat * math.pi / 180)
        tx = s * er * math.pi * lon / 180.0
        ty = s * er * np.log(np.tan((90.0 + lat) * math.pi / 360.0))
        return tx, ty

    def run(self):
        self.readbag()
        self.gps2orientation()
        self.compare()

        print 'base lat: ', self.baseLat
        print 'octopus average gps_yaw: ', np.mean(self.data['gps_yaw'])
        self.plot()


if __name__ == '__main__':
    bag = sys.argv[1]
    ori = Orientation(bag)
    ori.run()
