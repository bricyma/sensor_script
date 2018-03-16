#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy

map_file = "test.hdmap"

data = {'test': {'x': [], 'y': [], 'yaw': []},
        'map': {'x': [], 'y': [], 'yaw': []}}
with open(map_file, 'rb') as f:
    submap = f.read()
map = TSMap(submap)


def parse():
    count = 0
    with open('gps2.txt') as f:
        for line in f:
            count += 1
            if count < 80000:
                continue
            if count > 300000:
                break
            a = line.split(' ')
            x, y = latlon2xy(float(a[0]), float(a[1]))
            data['test']['yaw'].append(float(a[2]))
            data['test']['x'].append(float(a[0]))
            data['test']['y'].append(float(a[1]))
            p = Point3d(x, y)
            ref_p = map.get_ref_pt(p)
            data['map']['x'].append(ref_p.x)
            data['map']['y'].append(ref_p.y)


def plot():
    for d in data:
        for k in data[d]:
            data[d][k] = np.array(data[d][k])
    x2 = data['map']['x'][1:]
    x = data['map']['x'][:-1]
    y2 = data['map']['y'][1:]
    y = data['map']['y'][:-1]
    data['map']['yaw'] = np.arctan2(x2 - x, y2 - y) * 180 / np.pi + 360

    data['map']['yaw'] = np.append(data['map']['yaw'], data['map']['yaw'][-1])
    plt.subplot(211)
    plt.plot(data['map']['yaw'], 'r', label='map')
    plt.plot(data['test']['yaw'], 'b', label='test')
    plt.legend(loc='upper left')

    plt.subplot(212)
    diff = data['map']['yaw'] - data['test']['yaw']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label='map - test')
    plt.legend(loc='upper left')
    plt.show()


parse()
plot()
