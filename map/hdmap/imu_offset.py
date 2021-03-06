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

# yaw is derived from x,y path in global coordination
# yaw2 is derived from Yaw conversion by calculating the offset between north and true north
# azimuth is from azimuth in INSPVAX
data = {'test': {'x': [], 'y': [], 'azimuth': [], 'yaw': [], 'bestvel': [], 'yaw2': []},
        'map': {'x': [], 'y': [], 'yaw': [], 'yaw2': []}}
delta_pos = {'x': [], 'y': []}
baseLat = 32.75707
baseLon = -111.55757
yaw_conver = YawConversion()
loc_correction = []
window_size = 10000
yaw_diff = deque(maxlen=window_size)  
yaw_diff_window = [0] # yaw angle difference between inspvax and bestvel
yaw_diff_window2 = [0] # yaw angle difference between azimuth and local yaw


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
                lat, lon, azimuth, bestvel = float(a[0]), float(a[1]), float(a[2]), float(a[5])
                x, y = latlon2xy(lat, lon)
                p = Point3d(x, y)
                ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(azimuth)
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                data['test']['bestvel'].append(bestvel)
                data['map']['x'].append(ref_p.x)
                data['map']['y'].append(ref_p.y)
            except:
                pass
    # global to local, x,y,x2,y2 => lat,lng => x',y', x2',y2' => yaw2
    for k in data:
        for i in range(0, len(data[k]['x'])-1):
            x = data[k]['x'][i]
            y = data[k]['y'][i]
            x2 = data[k]['x'][i+1]
            y2 = data[k]['y'][i+1]
            yaw = yaw_conver.pos_conversion3(x,y,x2,y2)
            if yaw < -10:
                yaw += 360
            data[k]['yaw2'].append(yaw)
        data[k]['yaw2'].append(data[k]['yaw2'][-1])
        
    for d in data:
        for k in data[d]:
            data[d][k] = np.array(data[d][k])

    for k in data:
        x2 = data[k]['x'][1:]
        x = data[k]['x'][:-1]
        y2 = data[k]['y'][1:]
        y = data[k]['y'][:-1]
        data[k]['yaw'] = np.rad2deg(np.arctan2(x2 - x, y2 - y))
        print 'aaa', k,  data[k]['yaw']
        for i in range(0, len(data[k]['yaw'])):
            if data[k]['yaw'][i] < 0:
                data[k]['yaw'][i] += 360
        data[k]['yaw'] = np.append(data[k]['yaw'], data[k]['yaw'][-1])

    # calculate yaw diff between inspvax and bestvel
    diff = data['test']['bestvel'] - data['test']['azimuth']
    for i in range(0, len(diff)):
        if (abs(diff[i]) < 1):
            yaw_diff.append(diff[i])
        yaw_diff_window.append(np.mean(yaw_diff))

    yaw_diff.clear()
    diff = data['test']['yaw2'] - data['test']['azimuth']
    for i in range(0, len(diff)):
        if (abs(diff[i]) < 1):
            yaw_diff.append(diff[i])
        yaw_diff_window2.append(np.mean(yaw_diff))


# filter those very large or small value
def check(diff):
    for i in range(0, len(diff)):
        if diff[i] < -300:
            diff[i] += 360
    diff[abs(diff)>2] = 0
    return diff

def plot():
    # check inspvax and bestvel
    plt.subplot(311)
    diff = data['test']['bestvel'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r', label='bestvel - azimuth')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')

    plt.subplot(312)
    plt.plot(yaw_diff_window, 'r.', label = 'window bestvel - azimuth')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')

    plt.subplot(313)
    plt.plot(yaw_diff_window2, 'r.', label = 'window local yaw - azimuth')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')
    plt.show()

    plt.subplot(511)
    plt.plot(data['map']['yaw2'], 'r', label='map_gps')

    plt.plot(data['test']['yaw'], 'b', label='test_gps')
    plt.plot(data['test']['azimuth'], 'g', label='test_azimuth')
    plt.legend(loc='upper left')

    plt.subplot(512)
    diff = data['test']['yaw2'] - data['test']['yaw']
    for i in range(0, len(diff)):
        if diff[i] < -300:
            diff[i] += 360
    diff[abs(diff)>2] = 0
    plt.plot(diff, 'r.', label = 'local yaw - global yaw')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    print 'mean of corrected yaw: ', np.mean(diff)
    plt.legend(loc='upper left')

    plt.subplot(513)
    diff = data['test']['yaw2'] - data['test']['azimuth']
    for i in range(0, len(diff)):
        if diff[i] < -300:
            diff[i] += 360
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label = 'local yaw - azimuth')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    print 'mean of local yaw-azimuth: ', np.mean(diff)

    plt.legend(loc='upper left')

    plt.subplot(514)
    diff = data['test']['yaw'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label='global yaw - azimuth')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')

    # bestvel and azimuth
    plt.subplot(515)
    diff = data['test']['bestvel'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
  
    plt.plot(diff, 'r', label='bestvel - azimuth')
    print 'mean of bestvel-azimuth: ', np.mean(diff)
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')
    plt.show()

    # show bestvel
    plt.plot(data['test']['bestvel'], 'r.', label='bestvel')
    plt.plot(data['test']['azimuth'], 'b.', label='azimuth')
    plt.legend(loc='upper left')
    plt.show()

    # show map
    plt.subplot(211)
    diff = data['test']['azimuth'] - data['map']['yaw2']
    diff = check(diff)
    plt.plot(diff, 'r.', label='azimuth - map yaw2')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')

    plt.subplot(212)
    diff = data['test']['bestvel'] - data['map']['yaw2']
    diff = check(diff)
    plt.plot(diff, 'r.', label='bestvel - map yaw2')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')
    plt.show()

    # bestvel match local gps
    diff = data['test']['yaw2'] - data['test']['bestvel']
    diff = check(diff)
    plt.plot(diff, 'r.', label='local yaw - bestvel')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')
    plt.show()

    # local gps match local map
    diff = data['test']['yaw2'] - data['map']['yaw2']
    diff = check(diff)
    plt.plot(diff, 'r.', label='local yaw - local map')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')
    plt.show()


parse()
plot()
