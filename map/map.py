#!/usr/bin/env python
from vsimple.map import Map
import ujson as json
import time
import sys

file_novatel = '../../dataset/'
file_novatel += sys.argv[1]
file_ublox = '../../dataset/'
file_ublox += sys.argv[2]
novatel_lat = []
novatel_lng = []
ublox_lat = []
ublox_lng = []
nova_k = 10
ublox_k = 2

def readData():
    input_file_novatel = open(file_novatel, 'r')
    input_file_ublox = open(file_ublox, 'r')
    for line in input_file_novatel:
        l = line.split(',')
        print type(l[1])
        novatel_lat.append(l[1])
        novatel_lng.append(l[0])
    for line in input_file_ublox:
        l = line.split(',')
        ublox_lat.append(l[1])
        ublox_lng.append(l[0])


def callback():
    global m1, m2
    m1.clear()
    m2.clear()
    print len(novatel_lat), len(ublox_lat)
    m1.go(novatel_lat[0], novatel_lng[0], 15)
    m2.go(ublox_lat[0], ublox_lng[0], 15)

    for i in xrange(0, len(novatel_lat)/nova_k, 1):
        m1.scatter(novatel_lat[nova_k*i], novatel_lng[nova_k*i], 'green')
        # if i > len(novatel_lat)/nova_k/4:
        if (ublox_k * i +1 < len(ublox_lat)):
            m2.scatter(ublox_lat[ublox_k*i], ublox_lng[ublox_k*i], 'red')
        time.sleep(0.005)

    # for i in range(0, len(ublox_lng)):
    #     time.sleep(0.05)



if __name__ == '__main__':

    m1 = Map()
    m2 = Map()
    readData()
    callback()


