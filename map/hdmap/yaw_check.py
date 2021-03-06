#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from llh2enu.llh2enu_gps_transformer import *
import sys
from yaw_conversion1 import *
from collections import deque
# map_file = "I-10_Phoenix2Tucson_20180315.hdmap"
map_file = "I-10_Tucson2Phoenix_20180323.hdmap"
gps_file = sys.argv[1]
filter_enable = False

# yaw is derived from x,y path
# map yaw2 is derived from Yaw conversion by calculating the offset between north and true north
# test yaw2 is derived from old data correction 
# azimuth is from azimuth in INSPVAX

data = {'map': {'x': [], 'y': [], 'yaw': [], 'yaw2': []}, 'test':{'x': [], 'y': [], 'yaw': [], 'azimuth': [], 'yaw2': []}}
cor = []
baseLat = 32.75707
baseLon = -111.55757
yaw_conver = YawConversion()
with open(map_file, 'rb') as f:
    submap = f.read()
map = TSMap(submap)
conver = YawConversion()

def parse():
    global cor
    count = 0
    with open(gps_file) as f:
        for line in f:
            try:
                count += 1
                a = line.split(' ')
                lat, lon, azimuth = float(a[0]), float(a[1]), float(a[2])
                x, y = latlon2xy(lat, lon)
                p = Point3d(x, y)
                ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(azimuth)
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                # print ref_p.x
                data['map']['x'].append(ref_p.x)
                data['map']['y'].append(ref_p.y)
            except:
                pass

    for d in data:
        for k in data[d]:
            data[d][k] = np.array(data[d][k])

    for k in data:
        x2 = data[k]['x'][1:]
        x = data[k]['x'][:-1]
        y2 = data[k]['y'][1:]
        y = data[k]['y'][:-1]
        data[k]['yaw'] = np.rad2deg(np.arctan2(x2 - x, y2 - y))
        for i in range(0, len(data[k]['yaw'])):
            if data[k]['yaw'][i] < 0:
                data[k]['yaw'][i] += 360
        data[k]['yaw'] = np.append(data[k]['yaw'], data[k]['yaw'][-1])

    # print baseLon
    # print cor
    for i in range(0, len(data['map']['x'])-1):
        map_yaw = data['map']['yaw'][i]
        corr_value = conver.yaw_corr(map_yaw, data['test']['azimuth'][i])
        cor.append(corr_value + data['test']['azimuth'][i])
    
    cor.append(cor[-1])
    cor = np.array(cor)


def plot():
    plt.subplot(211)
    plt.plot(data['map']['yaw'], 'r', label='map yaw')
    plt.plot(data['test']['azimuth'], 'g', label='test azimuth')
    plt.legend(loc='upper left')
    
    plt.subplot(212)
    diff = data['map']['yaw'] - data['test']['azimuth']
    diff[abs(diff)>2] = 0
    diff2 = data['map']['yaw'] - cor
    diff2[abs(diff2)>2] = 0
    plt.plot(diff, 'r', label='global map - novatel')
    plt.plot(diff2, 'b', label='local map - novatel')
    # plt.legend(loc='upper left')
    plt.show()


parse()
plot()