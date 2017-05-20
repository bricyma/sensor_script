#!/usr/bin/env python
from vsimple.map import Map
import numpy as np
import sys
import math as m
import matplotlib.pyplot as plt


file_novatel = '../../dataset/'
file_novatel += sys.argv[1]
file_ublox = '../../dataset/'
file_ublox += sys.argv[2]
novatel_lat = []
novatel_lng = []
ublox_lat = []
ublox_lng = []

novatel_t = []
novatel_x = []
novatel_y = []
ublox_t = []
ublox_x = []
ublox_y = []

diff = []
nova_k = 10
ublox_k = 2
lat0 = 32.8108021


def readData():
    input_file_novatel = open(file_novatel, 'r')
    input_file_ublox = open(file_ublox, 'r')
    for line in input_file_novatel:
        l = line.split(',')
        novatel_t.append(int(l[0]))
        transform_xy = transform(float(l[2]), float(l[1]))
        novatel_x.append(transform_xy[0])
        novatel_y.append(transform_xy[1])
    for line in input_file_ublox:
        l = line.split(',')
        ublox_t.append(int(l[0]))
        transform_xy = transform(float(l[2]), float(l[1]))
        ublox_x.append(transform_xy[0])
        ublox_y.append(transform_xy[1])
    print 'ok'
    print len(ublox_t), len(novatel_t)

def transform(lat, lon):
    er = 6378137.0
    s = np.cos(lat0 * m.pi/180)
    tx = s * er * m.pi * lon / 180.0
    ty = s * er * np.log(np.tan((90.0 + lat) * m.pi / 360.0))
    l = [tx, ty]
    return l


def compare():
    ublox_index = 0
    for i in range(0, len(novatel_t)):
        if ublox_t[ublox_index] == novatel_t[i] or ublox_t[ublox_index] < novatel_t[i]:
            diff.append(m.sqrt(((novatel_x[i] - ublox_x[ublox_index])**2 + (novatel_y[i] - ublox_y[ublox_index])**2)))
            ublox_index += 1
        if ublox_index == len(ublox_t):
            break

def plot():
    # plt.plot(ublox_t, 'y', label='ublox time')
    # plt.plot(novatel_t, 'r', label='novatel time')
    y2 = [3] * len(diff)

    plt.plot(y2, 'r', label='limit')
    plt.plot(diff, 'b', label='difference')
    plt.legend(loc='upper left')
    plt.xlabel('stamp')
    plt.ylabel('position accuracy(m)')
    plt.title("the Accuracy of Ublox EVK-M8U GPS 2d Position")
    plt.show()


if __name__ == '__main__':
    readData()
    compare()
    plot()

    print 'ublox length: ', len(ublox_t)
    print len(diff)
    print 'average: ', np.mean(diff)
    print 'min: ', np.min(diff)
    print 'max: ', np.max(diff)
    print 'std: ', np.std(diff)

