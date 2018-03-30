#!/usr/bin/env python

# usage: python yaw_analyzer.py
# YawAnalyzer is used to get the relationship between azimuth from INSPVAX, 
# trk_gnd from BESTVEL and local yaw from coordinate transformation.
# input: 
# gps file containing latitude, longitude, azimuth, trk_gnd
# hdmap
# output: figure is stored in folder figure
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
import sys
from yaw_conversion1 import *
from collections import deque
import time
import glob

class YawAnalyzer:
    def __init__(self, data):
        start_time = time.time()
        # map_file = "I-10_Phoenix2Tucson_20180315.hdmap"
        map_file = "I-10_Tucson2Phoenix_20180327.hdmap"
        # yaw is derived from x,y path in global coordination
        # yaw2 is derived from Yaw conversion by calculating the offset between north and true north
        # azimuth is from azimuth in INSPVAX
        self.gps_data = data
        self.data = {'x': [], 'y': [], 'azimuth': [], 'trk_gnd': [], 'yaw': [], 'yaw2': []}
        self.yaw_conver = YawConversion()

        # yaw1: inspvax - local_yaw
        # yaw2: bestvel - local_yaw
        # yaw3: inspvax - bestvel
        self.data_window = {'d_yaw1': [], 'd_yaw2': [], 'd_yaw3': []}
        self.data_no_window = {'d_yaw1': [], 'd_yaw2': [], 'd_yaw3': []}

        with open(map_file, 'rb') as f:
            submap = f.read()
        self.map = TSMap(submap)
        
    def parse(self):
        # print type(self.data['x'])
        for i in range(0, len(self.gps_data['lat'])):
            try:
                lat, lon, azimuth, trk_gnd = self.gps_data['lat'][i], self.gps_data['lon'][i], \
                                            self.gps_data['azimuth'][i], self.gps_data['trk_gnd'][i],  
                x, y = latlon2xy(lat, lon)
                p = Point3d(x, y)   
                ref_p = self.map.get_ref_pt(p)
                self.data['azimuth'].append(azimuth)
                self.data['trk_gnd'].append(trk_gnd)
                self.data['x'].append(x)                    
                self.data['y'].append(y)
                # print self.data
            except:
                pass
        # global to local, x,y,x2,y2 => lat,lng => x',y', x2',y2' => yaw2
        for i in range(0, len(self.data['x'])-1):
            x = self.data['x'][i]
            y = self.data['y'][i]
            x2 = self.data['x'][i+1]
            y2 = self.data['y'][i+1]
            yaw = self.yaw_conver.pos_conversion3(x,y,x2,y2)
            if yaw < -10:
                yaw += 360
            self.data['yaw2'].append(yaw)
        self.data['yaw2'].append(self.data['yaw2'][-1])
            
        for k in self.data:
            self.data[k] = np.array(self.data[k])

        # calculate window sized yaw diff between inspvax, bestvel and local yaw
        window_size = 20000
        yaw_diff = deque(maxlen=window_size)  
        for d_window in self.data_window:
            sum = 0
            if d_window == 'd_yaw1':
                diff = self.data['azimuth'] - self.data['yaw2']
            elif d_window == 'd_yaw2':
                diff = self.data['trk_gnd'] - self.data['yaw2']
            else:
                diff = self.data['azimuth'] - self.data['trk_gnd']
            diff2 = [0]
            # threshold: 1
            # filter those data >= 1
            for i, d in enumerate(diff):
                if abs(d) < 1:
                    diff2.append(d)
                elif d > 1:
                    diff2.append(diff2[i-1])
                else:
                    diff2.append(diff2[i-1])
            # data_no_win has window size 1
            self.data_no_window[d_window] = diff2
            for i in range(0, len(diff2)):
                sum += diff2[i]
                if i >= window_size:
                    sum -= diff2[i-window_size]
                    ave = sum/window_size
                else:
                    ave = sum/(i+1)
                self.data_window[d_window].append(ave)
        return self.data_window, self.data_no_window
    
# if __name__ == '__main__':
#     data_path = 'gps_data/'
#     files_name = sorted(glob.glob(data_path + "*.txt"))
#     for file in files_name:
#         print 'aaa'
#         analyzer = YawAnalyzer(file)
#     