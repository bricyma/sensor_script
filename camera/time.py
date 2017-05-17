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

gps_time_sec = []
ecu_time_sec = []
time = []

for topic, msg, t in bag.read_messages(topics=['/camera9/image_color/compressed']):
    time.append(msg.header.stamp.nsecs)

bag.close()
# plt.plot(gps_time_sec, 'b', label='gps')
# plt.plot(ecu_time_sec, 'g', label='ecu')
plt.plot(time, 'r', label='camera nsec')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.title("Time")
plt.show()



