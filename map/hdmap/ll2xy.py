#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from llh2enu.llh2enu_gps_transformer import *
import sys


baseLat1 = 32.75707
baseLon1 = -111.55757

baseLat2 = 32.262478
baseLon2 = -111.027631
gps_file = sys.argv[1]

# yaw is derived from x,y path
data = {'base1': {}, 'base2': {}}
for base in data:
    data[base] = {'x': [], 'y': [], 'yaw': []}


transform = gps_transformer()


def RAD2DEG(rad):
    return rad * 180 / np.pi

def parse():
    count = 0
    x_0, y_0 = transform.llh2enu_5(
        baseLat1, baseLon1, 0, baseLat1, baseLon1, 0)
    x2_0, y2_0 = transform.llh2enu_5(
        baseLat2, baseLon2, 0, baseLat2, baseLon2, 0)

    x_first, y_first = 0, 0
    x2_first, y2_first = 0, 0
    first_flag = True
    with open(gps_file) as f:
        for line in f:
            try:
                count += 1
                a = line.split(' ')
                lat, lng = float(a[0]), float(a[1])
                x, y = transform.llh2enu_5(
                    lat, lng, 0, baseLat1, baseLon1, 0)
                x2, y2 = transform.llh2enu_5(
                    lat, lng, 0, baseLat2, baseLon2, 0)
                
                if first_flag:
                    x_first, y_first = x, y
                    x2_first, y2_first = x2, y2
                    first_flag = False

                x2 = x2 - (x2_first - x_first)
                y2 = y2 - (y2_first - y_first)
                data['base1']['x'].append(x)
                data['base1']['y'].append(y)
                data['base2']['x'].append(x2)
                data['base2']['y'].append(y2)
            except:
                    pass
    for k in data:
        for i in data[k]:
            data[k][i] = np.array(data[k][i])   
    for k in data:
        x2 = data[k]['x'][1:]
        x = data[k]['x'][:-1]
        y2 = data[k]['y'][1:]
        y = data[k]['y'][:-1]
        data[k]['yaw'] = RAD2DEG(np.arctan2(x2 - x, y2 - y))


def plot():
    print data['base1']['x']
    print data['base2']['x']
    plt.subplot(311)
    plt.plot(data['base1']['x'], data['base1']['y'], 'r.', label='base1')
    plt.plot(data['base2']['x'], data['base2']['y'], 'g.', label='base2')
    plt.legend(loc='upper left')

    plt.subplot(312)
    plt.plot(data['base1']['yaw'],'r.', label='base1')
    plt.plot(data['base2']['yaw'],'g.', label='base2')
    plt.legend(loc='upper left')

    plt.subplot(313)
    plt.plot(data['base1']['yaw'] - data['base2']['yaw'], 'r.', label='diff')
    plt.legend(loc='upper left')
    plt.show()

parse()
plot()
