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
        self.north_vel= []
        self.east_vel = []
        self.yaw = []
        self.ori_vel = []  # orientation calculated from east_velocity and north_velocity
        self.diff = []   # yaw - orientation_from
        self.data = {}
        self.baseLat = 0.693143165823  #caofeidian
        self.baseLon = 2.04736661229
        self.transform = gps_transformer()
        self.baseLat = self.DEG2RAD(39.03775082210) # wuhudao
        self.baseLon = self.DEG2RAD(118.43091220755)

        self.baseLat = self.RAD2DEG(self.baseLat)
        self.baseLon = self.RAD2DEG(self.baseLon)

        self.baseLat = 39.714178  # beijing
        self.baseLon = 117.305466
        print 'baseLat: ', self.baseLat
        print 'baseLon: ', self.baseLon


    def readbag(self):
        self.data['lat'] = []
        self.data['lon'] = []
        self.data['yaw'] = []
        self.data['cal_diff'] = []
        self.data['best_yaw'] = []
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/bestvel']):
            self.data['best_yaw'].append(msg.trk_gnd)
        for topic, msg, t in self.bag.read_messages(topics=['/calibrate/diff']):
            if msg.data > 2 or msg.data < -2:
                msg.data = 0
            self.data['cal_diff'].append(msg.data)

        first_flag = True # choose the start point as the base
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            if first_flag:
                self.baseLat = msg.latitude
                self.baseLon = msg.longitude
                first_flag = False
            self.north_vel.append(msg.north_velocity)
            self.east_vel.append(msg.east_velocity)
            self.data['yaw'].append(msg.azimuth)
            self.data['lat'].append(msg.latitude)
            self.data['lon'].append(msg.longitude)
        self.bag.close()

    def gps2orientation(self):
        self.data['gps_yaw'] = []
        for i, lat in enumerate(self.data['lat']):
            if i + 2 > len(self.data['lat']):
                break
            lon = self.data['lon'][i]
            # method1: octopus
            x, y = self.transform.llh2enu_1(lat, lon, 0, self.baseLat, self.baseLon, 0)
            #x, y = self.latlon2xy2(lat, lon)

            # method2: kitti
            xx, yy = self.latlon2xy(lat, lon)

            lat2 = self.data['lat'][i+1]
            lon2 = self.data['lon'][i+1]

            # x2, y2 = self.latlon2xy2(lat2, lon2)
            x2, y2 = self.transform.llh2enu_1(lat2, lon2, 0, self.baseLat, self.baseLon, 0)

            gps_yaw = self.RAD2DEG(np.arctan2(x2-x, y2-y))
            if gps_yaw < 0:
                gps_yaw += 360
            self.data['gps_yaw'].append(gps_yaw)

    def orientation_from_velocity(self):
        for i in range(0, len(self.north_vel)):
            if np.arctan2(self.east_vel[i], self.north_vel[i]) < 0:
                cur_ori = np.arctan2(self.east_vel[i], self.north_vel[i]) * 180 / np.pi + 360
            else:
                cur_ori = np.arctan2(self.east_vel[i], self.north_vel[i]) * 180 / np.pi

            self.ori_vel.append(cur_ori)

    def compare(self):
        yaw1 = np.array(self.data['yaw'])
        yaw2 = np.array(self.data['gps_yaw'])
        self.diff = yaw2 - yaw1[:-1]
        for i, unit in enumerate(self.diff):
            if unit > 2 or unit < -2:
                self.diff[i] = 0
        print yaw2, yaw1
        print 'mean diff: ', np.mean(self.diff)

    def plot(self):
        plt.subplot(411)
        plt.plot(self.data['yaw'], 'r', label='yaw')
        plt.plot(self.data['gps_yaw'], 'b', label='orientation from gps')
        plt.legend(loc='upper left')
        plt.subplot(412)
        plt.plot(self.diff, 'r', label='gps_yaw-inspvax')
        plt.legend(loc='upper left')
        plt.subplot(413)
        # plt.plot(self.data['cal_diff'], 'b', label='bestvel-inspvax ')
        plt.plot(self.data['best_yaw'][::2], 'b', label='bestvel')
        plt.plot(self.data['yaw'][::5],'r', label='inspvax yaw')
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
        s = np.cos(lat0 * math.pi / 180)
        tx = s * er * math.pi * lon / 180.0
        ty = s * er * np.log(np.tan((90.0 + lat) * math.pi / 360.0))
        return tx, ty

    # there is some problem in it
    # there is some problem in it
    def latlon2xy2(self, lat_, lon_):
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
