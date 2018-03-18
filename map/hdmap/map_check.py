#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from llh2enu.llh2enu_gps_transformer import *
map_file = "I-10_Tucson2Phoenix_20180316.hdmap"

# yaw is derived from x,y path
# azimuth is from azimuth in INSPVAX
data = {'test': {'x': [], 'y': [], 'azimuth': [], 'yaw': []},
        'map': {'x': [], 'y': [], 'yaw': []}}

baseLat = 32.75707
baseLon = -111.55757
trans = gps_transformer()

with open(map_file, 'rb') as f:
    submap = f.read()
map = TSMap(submap)

def parse():
    count = 0
    with open('gps2.txt') as f:
        for line in f:
            try:
                count += 1
                a = line.split(' ')
                x, y = latlon2xy(float(a[0]), float(a[1]))
                # x, y = trans.llh2enu_2(float(a[0]), float(a[1]), 0, baseLat, baseLon, 0)
                p = Point3d(x, y)
                ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(float(a[2]))
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                data['map']['x'].append(ref_p.x)
                data['map']['y'].append(ref_p.y)
            except:
                pass

def RAD2DEG(rad):
    return rad * 180 / np.pi

def plot():
    for d in data:
        for k in data[d]:
            data[d][k] = np.array(data[d][k])

    for k in data:
        x2 = data[k]['x'][1:]
        x = data[k]['x'][:-1]
        y2 = data[k]['y'][1:]
        y = data[k]['y'][:-1]
        if k == 'test':
            data[k]['yaw'] = RAD2DEG(np.arctan2(x2 - x, y2 - y))
        else:
            data[k]['yaw'] = RAD2DEG(np.arctan2(x2 - x, y2 - y))
        for i in range(0, len(data[k]['yaw'])):
            if data[k]['yaw'][i] < 0:
                data[k]['yaw'][i] += 360
        data[k]['yaw'] = np.append(data[k]['yaw'], data[k]['yaw'][-1])

    plt.subplot(411)
    plt.plot(data['map']['yaw'], 'r', label='map_gps')
    plt.plot(data['test']['yaw'], 'b', label='test_gps')
    plt.plot(data['test']['azimuth'], 'g', label='test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(412)
    diff = data['map']['yaw'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label='map - test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(413)
    diff = data['map']['yaw'] - data['test']['yaw']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label='map - test_gps')
    plt.legend(loc='upper left')

    plt.subplot(414)
    diff = data['test']['yaw'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label='test_gps - test_azimuth')
    plt.legend(loc='upper left')




    plt.show()


parse()
plot()