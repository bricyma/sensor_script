#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import sys
from yaw_conversion1 import *


gps_file = sys.argv[1]
data = {'dE':[], 'dN': [], 'dYaw': []}
conver = YawConversion()
def parse():
    with open(gps_file) as f:
        for line in f:
            a = line.split(' ')
            lat, lon = float(a[0]), float(a[1]) 
            de, dn, dyaw = conver.pos_conversion2(lat, lon)
            data['dE'].append(de)
            data['dN'].append(dn)
            data['dYaw'].append(dyaw)


def plot():
    plt.plot(data['dE'], 'r', label='East error')
    plt.plot(data['dN'], 'b', label='North error')
    plt.plot(data['dYaw'], 'g', label='Yaw error')
    plt.legend(loc='upper left')
    plt.show()


parse()
plot()
