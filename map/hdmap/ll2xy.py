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
# azimuth is from azimuth in INSPVAX
data = {'test': {'x': [], 'y': [], 'x2': [], 'y2': [], 'azimuth': [], 'yaw': [], 'yaw2': []},
        'map': {'x': [], 'y': [], 'yaw': []}}
transform = gps_transformer()


def RAD2DEG():
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
    
                data['test']['azimuth'].append(float(a[2]))
                data['test']['bestvel'].append(float(a[3]))
                data['test']['x'].append(x)
                data['test']['y'].append(y)
                data['test']['x2'].append(x2)
                data['test']['y2'].append(y2)
            except:
                pass


def plot():
    plt.plot(data['test']['x'], data['test']['y'], 'r.', label='32.75707')
    plt.plot(data['test']['x2'], data['test']['y2'], 'g.', label='32.262478')

    plt.legend(loc='upper left')
    plt.show()


parse()
plot()
