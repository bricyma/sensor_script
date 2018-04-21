#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from gnss_transformer import GPSTransformer
from collections import deque
from parse import DataParser


class INSAnalyzer:
    def __init__(self, info):
        parser = DataParser(info)
        insspd_data, inspvax_data, bestvel_data, pose_data, corrimu_data = parser.parse_insspd()
        self.data = {'insspd': {}, 'inspvax': {}, 'pose': {}}
        self.vehicle_transformer = GPSTransformer()
        self.corrimu_warp(corrimu_data)
        self.insspd_warp(insspd_data)
        self.pose_warp(pose_data)
        self.inspvax_warp(inspvax_data)
        self.list2array()
        self.plot(info['bag_name'], info['ts_begin'], info['ts_end'])

    def corrimu_warp(self, imu):
        self.data['corrimu'] = {'yaw_rate': []}
        for p in imu:
            self.data['corrimu']['yaw_rate'].append(p.yaw_rate)

    def pose_warp(self, pose):
        self.data['pose'] = {'yaw': []}
        for p in pose:
            self.data['pose']['yaw'].append(
                np.rad2deg(-p.pose.orientation.w + np.pi / 2))

    def insspd_warp(self, insspd):
        self.data['insspd'] = {'yaw': []}
        for p in insspd:
            self.data['insspd']['yaw'].append(p.track_ground)

    def inspvax_warp(self, inspvax):
        lat0, lon0 = 33, -111
        p0 = None
        self.data['inspvax'] = {'yaw': [], 'yaw_std': [], 'pitch': [],
                                'vel': [], 'ext_sol': [], 'local_yaw': [],
                                'altitude': []}
        for p in inspvax:
            self.data['inspvax']['altitude'].append(p.altitude)
            self.data['inspvax']['yaw'].append(p.azimuth)
            self.data['inspvax']['pitch'].append(p.pitch)
            
            self.data['inspvax']['vel'].append(
                np.sqrt(p.north_velocity ** 2 + p.east_velocity ** 2))
            self.data['inspvax']['ext_sol'].append(p.extended_status)
            self.data['inspvax']['yaw_std'].append(p.azimuth_std)
            lat, lon = p.latitude, p.longitude
            self.vehicle_transformer.set_base(lat, lon)
            enu_pt = self.vehicle_transformer.latlon2xy(np.array([lat, lon]))
            enu_pt0 = self.vehicle_transformer.latlon2xy(
                np.array([lat0, lon0]))
            delta = (np.array(enu_pt) - np.array(enu_pt0))
            yaw = np.rad2deg(np.arctan2(delta[0], delta[1]))
            if yaw < 0:
                yaw += 360
            # fix duplicate stamp
            if yaw == 0:
                yaw = self.data['inspvax']['local_yaw'][-1]
            self.data['inspvax']['local_yaw'].append(yaw)
            lat0, lon0 = lat, lon
            p0 = p

    def list2array(self):
        for item in ['insspd', 'pose', 'inspvax']:
            for k in self.data[item]:
                self.data[item][k] = np.array(self.data[item][k])
        # self.data['inspvax']['ext_sol'] = self.data['inspvax']['ext_sol'] & 192

    def plot(self, bag_name, ts_begin, ts_end):
        # enlarge the font size of figure
        font = {'family': 'normal',
                'weight': 'bold',
                'size': 14}
        plt.rc('font', **font)
        # check inspvax, bestvel, insspd difference
        fig = plt.figure()
        plt.subplot(411)
        plt.plot(self.data['inspvax']['local_yaw'], 'r', label='local yaw')
        plt.plot(self.data['insspd']['yaw'], 'b', label='insspd')
        plt.plot(self.data['inspvax']['yaw'], 'g', label='inspvax')
        plt.legend(loc='upper left')

        plt.subplot(412)
        diff = self.data['insspd']['yaw'] - self.data['inspvax']['yaw']
        for i, d in enumerate(diff):
            if d > 350:
                diff[i] -= 360
            elif d < -350:
                diff[i] += 360
        diff[abs(diff) > 1] = 0
        plt.plot(diff, 'r', label='insspd - inspvax')
        plt.plot([0] * len(diff), 'g', linewidth=3.0, label='0')
        plt.legend(loc='upper left')

        # 192: INS solution convergence flag & Doppler update
        # 64: INS solution convergence
        plt.subplot(413)
        # yaw diff 
        # yaw_rate = np.array(self.data['corrimu']['yaw_rate'])
        # diff_yaw_rate = (yaw_rate[1:] - yaw_rate[:-1])/0.02
        # plt.plot(diff_yaw_rate, 'r', label='yaw_rate')
        
        plt.plot(self.data['inspvax']['pitch'], 'r', label = 'pitch')
        # plt.plot(self.data['inspvax']['yaw_std'], 'r', label='inspvax yaw_std')
        # plt.plot(self.data['inspvax']['ext_sol'], 'r', label='extended_status')
        plt.legend(loc='upper left')

        plt.subplot(414)
        # plt.plot(self.data['inspvax']['vel'], 'r', label='velocity')
        plt.plot(self.data['inspvax']['altitude'], 'r', label='altitude')

        plt.legend(loc='upper left')

        fig.suptitle(bag_name + ' ' + ts_begin + '~' + ts_end, fontsize=20)
        plt.show()


if __name__ == '__main__':
    # bag_name = '2018-04-06-13-39-28'
    # bag_name = '2018-04-09-14-40-14'
    # bag_name = '2018-03-24-17-52-31'
    # bag_name = '2018-04-09-16-08-16'
    bag_name = '2018-04-18-10-42-02'
    # bag_name = '2018-04-09-16-08-16'
    # bag_name = '2018-04-09-14-40-14'
    # bag_name = '2018-04-19-13-30-26'

    ts_begin = '10:00 '
    ts_end = '120:00'
    info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
    analyzer = INSAnalyzer(info)
