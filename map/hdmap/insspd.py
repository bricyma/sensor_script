#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from llh2enu.llh2enu_gps_transformer import *
from collections import deque
from parse import DataParser

class INSAnalyzer:
    def __init__(self, info):
        parser = DataParser(info)
        insspd_data, inspvax_data, bestvel_data, pose_data = parser.parse_insspd()
        self.data = {'insspd': {}, 'inspvax':{}, 'pose': {}}
        self.insspd_warp(insspd_data)
        self.pose_warp(pose_data)
        self.inspvax_warp(inspvax_data)
        self.plot()
    def pose_warp(self, pose): 
        self.data['pose'] = {'yaw': []}
        for p in pose:
            self.data['pose']['yaw'].append(np.rad2deg(-p.pose.orientation.w + np.pi / 2))
    
    def insspd_warp(self, insspd):
        self.data['insspd'] = {'yaw': []}
        for p in insspd:
            self.data['insspd']['yaw'].append(p.track_ground)
    
    def inspvax_warp(self, inspvax):    
        self.data['inspvax'] = {'yaw': [], 'vel': []}
        for p in inspvax:
            self.data['inspvax']['yaw'].append(p.azimuth)
            self.data['inspvax']['vel'].append(np.sqrt(p.north_velocity ** 2 + p.east_velocity ** 2))



    def plot(self):
        # check inspvax, bestvel, insspd difference
        plt.subplot(311)
        plt.plot(self.data['pose']['yaw'], 'r', label='pose')
        plt.plot(self.data['insspd']['yaw'], 'b', label='insspd')
        plt.plot(self.data['inspvax']['yaw'], 'g', label='inspvax')
        plt.legend(loc='upper left')

        plt.subplot(312)
        plt.plot(self.data['inspvax']['vel'], 'r', label='velocity')
        plt.legend(loc='upper left')

        plt.show()

if __name__ == '__main__':
    # bag_name = '2018-04-06-13-39-28'
    bag_name = '2018-04-09-14-40-14'
    # bag_name = '2018-04-09-16-08-16'
    ts_begin = '0:00 '
    ts_end = '120:00'
    info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
    analyzer = INSAnalyzer(info)
