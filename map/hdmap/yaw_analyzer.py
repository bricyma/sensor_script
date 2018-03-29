#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
import sys
from yaw_conversion1 import *
from collections import deque
import time
import glob

class YawAnalyzer:
    def __init__(self, gps_file):
        start_time = time.time()

        # map_file = "I-10_Phoenix2Tucson_20180315.hdmap"
        map_file = "I-10_Tucson2Phoenix_20180327.hdmap"
        self.gps_file = gps_file
        # yaw is derived from x,y path in global coordination
        # yaw2 is derived from Yaw conversion by calculating the offset between north and true north
        # azimuth is from azimuth in INSPVAX
        self.data = {'test': {'x': [], 'y': [], 'azimuth': [], 'yaw': [], 'bestvel': [], 'yaw2': []}}
        baseLat = 32.75707
        baseLon = -111.55757
        self.yaw_conver = YawConversion()

        # yaw1: inspvax - local_yaw
        # yaw2: bestvel - local_yaw
        # yaw3: inspvax - bestvel
        self.data_window = {'test': {'d_yaw1': [], 'd_yaw2': [], 'd_yaw3': []}}


        with open(map_file, 'rb') as f:
            submap = f.read()
        self.map = TSMap(submap)
        self.parse()
        self.plot(self.data_window)
        print gps_file
        print("--- %s seconds ---" % (time.time() - start_time))

    def parse(self):
        count = 0
        print self.gps_file
        with open(self.gps_file) as f:
            for line in f:
                try:
                    a = line.split(' ')
                    lat, lon, azimuth, bestvel = float(a[0]), float(a[1]), float(a[2]), float(a[5])
                    x, y = latlon2xy(lat, lon)
                    p = Point3d(x, y)   
                    ref_p = self.map.get_ref_pt(p)
                    self.data['test']['azimuth'].append(azimuth)
                    self.data['test']['x'].append(x)                    
                    self.data['test']['y'].append(y)
                    self.data['test']['bestvel'].append(bestvel)
                except:
                    pass

        # global to local, x,y,x2,y2 => lat,lng => x',y', x2',y2' => yaw2
        for k in self.data:
            for i in range(0, len(self.data[k]['x'])-1):
                x = self.data[k]['x'][i]
                y = self.data[k]['y'][i]
                x2 = self.data[k]['x'][i+1]
                y2 = self.data[k]['y'][i+1]
                yaw = self.yaw_conver.pos_conversion3(x,y,x2,y2)
                if yaw < -10:
                    yaw += 360
                self.data[k]['yaw2'].append(yaw)
            self.data[k]['yaw2'].append(self.data[k]['yaw2'][-1])
            
        for d in self.data:
            for k in self.data[d]:
                self.data[d][k] = np.array(self.data[d][k])

        # calculate window sized yaw diff between inspvax, bestvel and local yaw
        window_size = 10000
        yaw_diff = deque(maxlen=window_size)  
        for d_window in self.data_window['test']:
            sum = 0
            if d_window == 'd_yaw1':
                diff = self.data['test']['azimuth'] - self.data['test']['yaw2']
            elif d_window == 'd_yaw2':
                diff = self.data['test']['bestvel'] - self.data['test']['yaw2']
            else:
                diff = self.data['test']['azimuth'] - self.data['test']['bestvel']

            diff2 = []
            for d in diff:
                if abs(d) < 1:
                    diff2.append(d)
            for i in range(0, len(diff2)):
                sum += diff2[i]
                if i >= window_size:
                    sum -= diff2[i-window_size]
                    ave = sum/window_size
                else:
                    ave = sum/(i+1)
                self.data_window['test'][d_window].append(-ave)

    def plot(self, data_window):
        # inspvax: azimuth
        # bestvel: trk_gnd
        # gps: local_yaw
        # map: local_map
        fig = plt.figure()
        fig.suptitle(self.gps_file, fontsize=20)

        # d_yaw1: inspvax - local_yaw
        # d_yaw2: bestvel - local_yaw
        # d_yaw3: inspvax - bestvel
        Len = len(data_window['test']['d_yaw1'])

        plt.subplot(311)
        plt.plot(data_window['test']['d_yaw1'], 'r.', label='inspvax - local yaw')
        plt.plot([0]*Len, 'g', linewidth=3,  label='0')
        plt.legend(loc=2, prop={'size': 9})

        plt.subplot(312)
        plt.plot(data_window['test']['d_yaw2'], 'r.', label = 'bestvel - local_yaw')
        plt.plot([0]*Len, 'g', linewidth=3,  label='0')
        plt.legend(loc=2, prop={'size': 9})

        plt.subplot(313)
        plt.plot(data_window['test']['d_yaw3'], 'r.', label = 'inspvax - bestvel')
        plt.plot([0]*Len, 'g', linewidth=3,  label='0')
        plt.legend(loc=2, prop={'size': 9})

        # plt.show()
        fig.savefig('figure/'+ self.gps_file.split('/')[1] + '.jpg')

if __name__ == '__main__':
    data_path = 'gps_data/'
    files_name = sorted(glob.glob(data_path + "*.txt"))
    for file in files_name:
        print 'aaa'
        analyzer = YawAnalyzer(file)
    