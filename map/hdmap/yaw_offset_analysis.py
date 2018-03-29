#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from llh2enu.llh2enu_gps_transformer import *
import sys
from yaw_conversion1 import *
from collections import deque
import time


start_time = time.time()

map_file = "I-10_Tucson2Phoenix_20180327.hdmap"
# map_file = "I-10_Phoenix2Tucson_20180327.hdmap"
gps_file = sys.argv[1]

# yaw is derived from x,y path in global coordination
# yaw2 is derived from Yaw conversion by calculating the offset between north and true north
# azimuth is from azimuth in INSPVAX
data = {'test': {'x': [], 'y': [], 'azimuth': [], 'yaw': [], 'bestvel': [], 'yaw2': []}}
baseLat = 32.75707
baseLon = -111.55757
yaw_conver = YawConversion()

# yaw1: inspvax - local_yaw
# yaw2: bestvel - local_yaw
# yaw3: inspvax - bestvel
data_window = {'test': {'d_yaw1': [], 'd_yaw2': [], 'd_yaw3': []}}


with open(map_file, 'rb') as f:
    submap = f.read()
map = TSMap(submap)


def parse():
    count = 0
    with open(gps_file) as f:
        for line in f:
            try:
                a = line.split(' ')
                lat, lon, azimuth, bestvel = float(a[0]), float(a[1]), float(a[2]), float(a[5])
                x, y = latlon2xy(lat, lon)
                p = Point3d(x, y)   
                ref_p = map.get_ref_pt(p)
                data['test']['azimuth'].append(azimuth)
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                data['test']['bestvel'].append(bestvel)
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

    # calculate window sized yaw diff between inspvax, bestvel and local yaw
    window_size = 10000
    yaw_diff = deque(maxlen=window_size)  
    print data['test']['azimuth']
    print data['test']['yaw2']
    print data['test']['bestvel']
    print 'diff: ', data['test']['azimuth'] - data['test']['yaw2']
    for d_window in data_window['test']:
        sum = 0
        if d_window == 'd_yaw1':
            diff = data['test']['azimuth'] - data['test']['yaw2']
        elif d_window == 'd_yaw2':
            diff = data['test']['bestvel'] - data['test']['yaw2']
        else:
            diff = data['test']['azimuth'] - data['test']['bestvel']
        diff2 = []
        plt.plot(diff, 'r')
        plt.show()
        for d in diff:
            if abs(d) < 2:
                diff2.append(d)
            else:
                print d
            if abs(abs(d) - 360) < 2:
                diff2.append(abs(d) - 360)
            
        for i in range(0, len(diff2)):
            sum += diff2[i]
            if i >= window_size:
                sum -= diff2[i-window_size]
                ave = sum/window_size
            else:
                ave = sum/(i+1)
            data_window['test'][d_window].append(-ave)
        print len(diff2), len(diff)       

def plot():
    # inspvax: azimuth
    # bestvel: trk_gnd
    # gps: local_yaw
    # map: local_map
    fig = plt.figure()
    fig.suptitle(gps_file, fontsize=20)

    # d_yaw1: inspvax - local_yaw
    # d_yaw2: bestvel - local_yaw
    # d_yaw3: inspvax - bestvel
    Len = len(data_window['test']['d_yaw1'])

    plt.subplot(311)
    plt.plot(data_window['test']['d_yaw1'], 'r.', label='inspvax - local yaw')
    plt.plot([0]*Len, 'g', linewidth=3,  label='0')
    # plt.legend(loc='upper left')
    plt.legend(loc=2, prop={'size': 9})

    plt.subplot(312)
    plt.plot(data_window['test']['d_yaw2'], 'r.', label = 'bestvel - local_yaw')
    plt.plot([0]*Len, 'g', linewidth=3,  label='0')
    plt.legend(loc=2, prop={'size': 9})

    plt.subplot(313)
    plt.plot(data_window['test']['d_yaw3'], 'r.', label = 'inspvax - bestvel')
    plt.plot([0]*Len, 'g', linewidth=3,  label='0')
    plt.legend(loc=2, prop={'size': 9})

    plt.show()
    fig.savefig('figure/'+ gps_file.split('/')[1] + '.jpg')


parse()

print("--- %s seconds ---" % (time.time() - start_time))

plot()

