#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from llh2enu.llh2enu_gps_transformer import *
import sys
from yaw_conversion1 import *

# map_file = "I-10_Phoenix2Tucson_20180315.hdmap"
map_file = "I-10_Tucson2Phoenix_20180323.hdmap"
gps_file = sys.argv[1]
filter_enable = False

# yaw is derived from x,y path
# yaw2 is derived from Yaw conversion by calculating the offset between north and true north
# azimuth is from azimuth in INSPVAX
data = {'test': {'x': [], 'y': [], 'azimuth': [], 'yaw': [], 'bestvel': [], 'yaw2': []},
        'map': {'x': [], 'y': [], 'yaw': []}}
delta_pos = {'x': [], 'y': []}
baseLat = 32.75707
baseLon = -111.55757
trans = gps_transformer()
yaw_conver = YawConversion()
loc_correction = []
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
                lat, lon, azimuth = float(a[0]), float(a[1]), float(a[2])
                x, y = latlon2xy(lat, lon)
                p = Point3d(x, y)
                ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(azimuth)
                data['test']['bestvel'].append(float(a[3]))
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                corr, azi = yaw_conver.inspvax_wrap(lat, lon, azimuth)
                data['test']['yaw2'].append(azi)
                loc_correction.append(-corr)
                data['map']['x'].append(ref_p.x)
                data['map']['y'].append(ref_p.y)
                delta_pos['x'].append(ref_p.x - x)
                delta_pos['y'].append(ref_p.y - y)
                
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
        data[k]['yaw'] = RAD2DEG(np.arctan2(x2 - x, y2 - y))
        print 'aaa', k,  data[k]['yaw']
        for i in range(0, len(data[k]['yaw'])):
            if data[k]['yaw'][i] < 0:
                data[k]['yaw'][i] += 360
        
        data[k]['yaw'] = np.append(data[k]['yaw'], data[k]['yaw'][-1])
    print data['map']['yaw'][222:55555]

def RAD2DEG(rad):
    return rad * 180 / np.pi

def yaw_conv(d):
    window = 10000
    diff_w = []
    sum = 0
    for i in range (0, len(d)):
        if d[i] > 2:
            continue
        if i < window:
            sum += d[i]
            diff_w.append(d[i])
        else:
            sum += d[i]
            sum -= d[i-window]
            ave = sum/window
            diff_w.append(ave)
    return np.array(diff_w)

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
    plt.subplot(511)
    plt.plot(data['map']['yaw'], 'r', label='map_gps')
    plt.plot(data['test']['yaw'], 'b', label='test_gps')
    plt.plot(data['test']['azimuth'], 'g', label='test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(512)
    
    diff = data['map']['yaw'] - data['test']['azimuth']
    diff2 = data['map']['yaw'] - data['test']['yaw2'] 
    diff[abs(diff) > 2] = 0
    diff2[abs(diff2) > 2] = 0
    
    # original
    plt.plot(diff, 'r', label='map - test_azimuth') 
    # localization yaw offset correction
    plt.plot(diff2, 'b', label='map - test_azimuth2')
    plt.legend(loc='upper left')
    
    plt.subplot(513)
    diff = data['test']['yaw'] - data['test']['azimuth']
    diff[abs(diff)>2] = 0
    diff_corr = yaw_conv(diff)
    # zhibei's yaw offset correction
    diff3 = data['map']['yaw'] - (data['test']['azimuth'] + diff_corr)    
    diff3[abs(diff3) > 2] = 0
    
    diff2 = data['map']['yaw'] - data['test']['yaw2'] 
    diff2[abs(diff2) > 2] = 0
    plt.plot(diff3, 'g', label='gps corrected map-azimuth')
    plt.plot(diff_corr, 'r', linewidth=4, label='gps correction')
    # localization yaw offset correction
    plt.plot(diff2, 'b', label='map - test_azimuth2')
    plt.legend(loc='upper left')
    
    # map yaw and test yaw
    # plt.subplot(513)
    # diff = data['map']['yaw'] - data['test']['yaw']
    # diff[abs(diff) > 2] = 0
    # # filter, smooth
    # if filter_enable:
    #     diff = savitzky_golay(diff, 2001, 3)
    # plt.plot(diff, 'r.', label='map - test_gps')
    # plt.legend(loc='upper left')

    # test yaw and azimuth
    plt.subplot(514)
    diff = data['test']['yaw'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0

    diff2 = yaw_conv(diff)

    plt.plot(diff, 'r.', label='test_gps - test_azimuth')
    plt.plot(diff2, 'b.', label='gps yaw correction')
    plt.plot(loc_correction, 'g.', label='localization correction')
    plt.legend(loc='upper left')
    
    # bestvel and azimuth
    plt.subplot(515)
    diff = data['test']['bestvel'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    # filter, smooth
    if filter_enable:
        diff = savitzky_golay(diff, 2001, 3)
    plt.plot(diff, 'r.', label='bestvel - azimuth')
    plt.legend(loc='upper left')

    plt.show()
    dx, dy = np.array(delta_pos['x']), np.array(delta_pos['y'])
    theta = data['test']['azimuth']
    dx_body = dx * np.cos(theta) - dy * np.sin(theta)
    dy_body = dx * np.sin(theta) + dy * np.cos(theta)
    dx_body[abs(dx_body)>2] = 0
    dy_body[abs(dy_body)>2] = 0
    plt.plot(dx_body, 'r', label='postion error on x')
    # plt.plot(dy_body, 'b', label='postion error on y')
    plt.legend(loc='upper left')
    plt.show()
parse()
plot()
