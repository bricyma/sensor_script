#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from llh2enu.llh2enu_gps_transformer import *
import sys

map_file = "I-10_Phoenix2Tucson_20180315.hdmap"
# map_file = "I-10_Tucson2Phoenix_20180316.hdmap"
gps_file = sys.argv[1]
# yaw is derived from x,y path
# azimuth is from azimuth in INSPVAX
data = {'test': {'x': [], 'y': [], 'azimuth': [], 'yaw': [], 'bestvel': []},
        'map': {'x': [], 'y': [], 'yaw': []}}

baseLat = 32.75707
baseLon = -111.55757
trans = gps_transformer()

with open(map_file, 'rb') as f:
    submap = f.read()
map = TSMap(submap)


def parse():
    count = 0
    with open(gps_file) as f:
        for line in f:
            try:
                count += 1
                a = line.split(' ')
                x, y = latlon2xy(float(a[0]), float(a[1]))
                p = Point3d(x, y)
                ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(float(a[2]))
                data['test']['bestvel'].append(float(a[3]))
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                data['map']['x'].append(ref_p.x)
                data['map']['y'].append(ref_p.y)
            except:
                pass


def RAD2DEG(rad):
    return rad * 180 / np.pi

# smooth the diff data
def savitzky_golay(y, window_size, order, deriv=0, rate=1):
    from math import factorial

    try:
        window_size = np.abs(np.int(window_size))
        order = np.abs(np.int(order))
    except ValueError, msg:
        raise ValueError("window_size and order have to be of type int")
    if window_size % 2 != 1 or window_size < 1:
        raise TypeError("window_size size must be a positive odd number")
    if window_size < order + 2:
        raise TypeError("window_size is too small for the polynomials order")
    order_range = range(order+1)
    half_window = (window_size -1) // 2
    # precompute coefficients
    b = np.mat([[k**i for i in order_range] for k in range(-half_window, half_window+1)])
    m = np.linalg.pinv(b).A[deriv] * rate**deriv * factorial(deriv)
    # pad the signal at the extremes with
    # values taken from the signal itself
    firstvals = y[0] - np.abs( y[1:half_window+1][::-1] - y[0] )
    lastvals = y[-1] + np.abs(y[-half_window-1:-1][::-1] - y[-1])
    y = np.concatenate((firstvals, y, lastvals))
    return np.convolve( m[::-1], y, mode='valid')

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

    plt.subplot(511)
    plt.plot(data['map']['yaw'], 'r', label='map_gps')
    plt.plot(data['test']['yaw'], 'b', label='test_gps')
    plt.plot(data['test']['azimuth'], 'g', label='test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(512)
    diff = data['map']['yaw'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    # filter, smooth
    diff = savitzky_golay(diff, 2001, 3)

    plt.plot(diff, 'r.', label='map - test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(513)
    diff = data['map']['yaw'] - data['test']['yaw']
    diff[abs(diff) > 2] = 0
    # filter, smooth
    diff = savitzky_golay(diff, 2001, 3)

    plt.plot(diff, 'r.', label='map - test_gps')
    plt.legend(loc='upper left')

    plt.subplot(514)
    diff = data['test']['yaw'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    # filter, smooth
    diff = savitzky_golay(diff, 2001, 3)
    plt.plot(diff, 'r.', label='test_gps - test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(515)
    diff = data['test']['bestvel'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    # filter, smooth
    diff = savitzky_golay(diff, 2001, 3)
    plt.plot(diff, 'r.', label='bestvel - azimuth')
    plt.legend(loc='upper left')

    plt.show()


parse()
plot()
