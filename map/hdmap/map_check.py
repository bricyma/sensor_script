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
with open(map_file, 'rb') as f:
    submap = f.read()
map = TSMap(submap)
trans = gps_transformer()

def parse():
    count = 0
    with open('gps2.txt') as f:
        for line in f:
            try:
                count += 1
                a = line.split(' ')
                # x, y = latlon2xy(float(a[0]), float(a[1]))
                x, y = trans.llh2enu_1(float(a[0]), float(a[1]), 0, baseLat, baseLon, 0)

                p = Point3d(x, y)
                ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(float(a[2]))
                data['test']['x'].append(float(a[0]))
                data['test']['y'].append(float(a[1]))
                data['map']['x'].append(ref_p.x)
                data['map']['y'].append(ref_p.y)
            except:
                pass


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
            data[k]['yaw'] = np.arctan2(y2 - y, x2 - x) * 180 / np.pi
        else:
            data[k]['yaw'] = np.arctan2(x2 - x, y2 - y) * 180 / np.pi
        for i in range(0, len(data[k]['yaw'])):
            if data[k]['yaw'][i] < 0:
                data[k]['yaw'][i] += 360
        data[k]['yaw'] = np.append(data[k]['yaw'], data[k]['yaw'][-1])

    plt.subplot(311)
    plt.plot(data['map']['yaw'], 'r', label='map_gps')
    plt.plot(data['test']['yaw'], 'b', label='test_gps')
    plt.plot(data['test']['azimuth'], 'g', label='test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(312)
    diff = data['map']['yaw'] - data['test']['yaw']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label='map - test')
    plt.legend(loc='upper left')

    plt.subplot(313)
    plt.plot(data['map']['x'], data['map']['y'], 'r', label='map')
    plt.plot(data['test']['x'], data['test']['y'], 'r', label='test')
    plt.legend(loc='upper left')

    plt.show()


parse()
plot()
