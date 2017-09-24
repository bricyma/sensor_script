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
EARTH_RADIUS = 6378137.0

class Orientation:
    def __init__(self, bagname):
        bag_path = '../../rosbag/'
        bag = bag_path + bagname
        self.bag = rosbag.Bag(bag)
        self.north_vel= []
        self.east_vel = []
        self.yaw = []
        self.ori_vel = []  # orientation calculated from east_velocity and north_velocity
        self.diff = []   # yaw - orientation_from
        self.data = {}
        self.baseLat = 0.693143165823
        self.baseLon = 2.04736661229

    def readbag(self):
        self.data['lat'] = []
        self.data['lon'] = []
        self.data['yaw'] = []
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
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
            x, y = self.latlon2xy(lat, lon)

            lat2 = self.data['lat'][i+1]
            lng2 = self.data['lon'][i+1]
            x2, y2 = self.latlon2xy(lat2, lng2)
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

        self.diff = yaw1[:-1] - yaw2
        print 'mean diff: ', np.mean(self.diff)

    def plot(self):
        plt.subplot(211)
        plt.plot(self.data['yaw'], 'r', label='yaw')
        plt.plot(self.data['gps_yaw'], 'b', label='orientation from gps')
        plt.legend(loc='upper left')
        plt.subplot(212)
        plt.plot(self.diff, 'r', label='diff')
        # plt.legend(loc='upper left')

        plt.show()


    # latitude longiutde to enu
    def DEG2RAD(self, x):
        return x / (180 / math.pi)

    def RAD2DEG(self, x):
        return x * (180 / math.pi)

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

    def run(self):
        self.readbag()
        self.gps2orientation()
        self.compare()
        self.plot()

if __name__ == '__main__':
    ori = Orientation("_2017-09-21-14-40-59_6.bag")
    ori.run()
