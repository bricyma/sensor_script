#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from llh2enu.llh2enu_gps_transformer import *
import sys
from yaw_conversion1 import *
from collections import deque

# map_file = "I-10_Phoenix2Tucson_20180315.hdmap"
map_file = "I-10_Tucson2Phoenix_20180327.hdmap"
gps_file = sys.argv[1]
filter_enable = False

# yaw is derived from x,y path in global coordination
# yaw2 is derived from Yaw conversion by calculating the offset between north and true north
# azimuth is from azimuth in INSPVAX
data = {'test': {'x': [], 'y': [], 'azimuth': [], 'yaw': [], 'bestvel': [], 'insspd': [], 'yaw2': []},
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
                lat, lon, azimuth, bestvel, insspd = float(a[0]), float(a[1]), float(a[2]), float(a[5]), float(a[6])
                x, y = latlon2xy(lat, lon)
                p = Point3d(x, y)
                # ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(azimuth)
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                data['test']['bestvel'].append(bestvel)
                data['test']['insspd'].append(insspd)
                # data['map']['x'].append(ref_p.x)
                # data['map']['y'].append(ref_p.y)
            except:
                pass
  
    for d in data:
        for k in data[d]:
            data[d][k] = np.array(data[d][k])

# filter those very large or small value
def check(diff):
    for i in range(0, len(diff)):
        if diff[i] < -300:
            diff[i] += 360
    diff[abs(diff)>2] = 0
    return diff



def plot():
    # check inspvax, bestvel, insspd difference
    plt.subplot(311)
    diff = data['test']['bestvel'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r', label='bestvel - azimuth')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')

    plt.subplot(312)
    diff = data['test']['bestvel'] - data['test']['insspd']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label = 'bestvel - insspd')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')

    plt.subplot(313)
    diff = data['test']['insspd'] - data['test']['azimuth']
    diff[abs(diff) > 2] = 0
    plt.plot(diff, 'r.', label = 'insspd - azimuth')
    plt.plot([0]*len(diff), 'g', linewidth=3,  label='0')
    plt.legend(loc='upper left')
    plt.show()


    plt.plot(data['test']['azimuth'], 'r.', label='inspvax')
    plt.plot(data['test']['bestvel'], 'b.', label='bestvel')
    plt.plot(data['test']['insspd'], 'g.', label='insspd')
    plt.legend(loc='upper left')
    plt.show()

parse()
plot()
