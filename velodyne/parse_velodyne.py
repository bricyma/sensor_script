# -*- coding: utf-8 -*-

import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np

bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)
sequence = 0
index = []

point_time_sec = []
point_time_nsec = []

packet_time_sec = []
packet_time_nsec = []

single_packet_time = []

# for topic, msg, t in bag.read_messages(topics=['/velodyne_points']):
#     point_time_sec.append(msg.header.stamp.secs)
#     point_time_nsec.append(msg.header.stamp.nsecs)

count = 0
before = 0
valid = []
for topic, msg, t in bag.read_messages(topics=['/velodyne_packets']):
    packet_valid = 0
    # print 'rotation', msg.packets[10].data
    # for i in range(0, 174):
    #     single_packet_time.append(msg.packets[i].stamp.secs)
    #     if msg.packets[i].stamp.secs > 0:
    #         packet_valid += 1
    # print packet_valid
    # valid.append(packet_valid)
    # print msg.header.stamp.to_sec() - before
    before = msg.header.stamp.to_sec()
    packet_time_sec.append(msg.header.stamp.secs)
    packet_time_nsec.append(msg.header.stamp.nsecs)

    count += 1
    if count == 1000:
        break


bag.close()
# plt.plot(point_time_sec, 'b', label='sec')
# plt.plot(point_time_nsec, 'g', label='nsec')
plt.plot(packet_time_sec, 'r', label='sec')
plt.plot(packet_time_nsec, 'r', label='nsec')
# plt.plot(valid, 'r')
# plt.plot(single_packet_time, 'r', label='nsec')


plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.title("Velodyne Time")
plt.show()












