#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import sys
from yaw_conversion1 import *

gps_file = sys.argv[1]
data = {'roll':[], 'error': [], 'pitch': [], 'azimuth': []}
conver = YawConversion()
def parse():
    with open(gps_file) as f:
        for line in f:
            try:
                a = line.split(' ')
                azimuth = float(a[2])
                roll = float(a[3])
                pitch = float(a[4])
                error = float(a[6])

                data['roll'].append(roll)
                data['pitch'].append(pitch)
                data['azimuth'].append(-azimuth + 90)
                data['error'].append(error)
            except:
                pass

def plot():
    plt.subplot(311)
    plt.plot(data['roll'], 'r', label='roll ')
    plt.plot(data['error'], 'g', label='error')
    plt.legend(loc='upper left')
    
    plt.subplot(312)
    plt.plot(data['error'], 'g', label='error')

    plt.plot(data['pitch'], 'r', label='pitch')
    plt.legend(loc='upper left')
    
    plt.subplot(313)
    plt.plot(data['azimuth'], 'g', label='azimuth')
    

    plt.legend(loc='upper left')
    plt.show()


parse()
plot()
