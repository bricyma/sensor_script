import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np


bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)

gps_file = open('gps_orientation_data.txt', 'w')
sequence = 0
index = []
lat_std = []
lng_std = []
alt_std = []
pos_2d_std = []

raw_lat_std = []
raw_lng_std = []
raw_pos_2d_std = []
raw_alt_std = []

raw_mean_accuracy = []
mean_accuracy = []
for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    lat_std.append(msg.latitude_std)
    lng_std.append(msg.longitude_std)
    pos_2d_std.append(m.sqrt(msg.latitude_std**2 + msg.longitude_std**2))
    alt_std.append(msg.altitude_std)
    index.append(sequence)
    sequence += 1

for topic, msg, t in bag.read_messages(topics=['/novatel_data/bestpos']):
    raw_lat_std.append(msg.latitude_std)
    raw_lng_std.append(msg.longitude_std)
    raw_pos_2d_std.append(m.sqrt(msg.latitude_std**2 + msg.longitude_std**2))
    raw_alt_std.append(msg.altitude_std)

m_accuracy = np.mean(pos_2d_std)
raw_m_accuracy = np.mean(raw_pos_2d_std)
for i in range(0, len(index)):
    mean_accuracy.append(m_accuracy)
    raw_mean_accuracy.append(raw_m_accuracy)

bag.close()
plt.ylim([0,1])
# plt.plot(pos_2d_std, 'b', label='RTK: real time')
# plt.plot(mean_accuracy, 'r', label='RTK: mean')
# plt.plot(raw_pos_2d_std, 'y', label='NO RTK: real time')
# plt.plot(raw_mean_accuracy, 'g', label='NO RTK: mean')
plt.plot(pos_2d_std, 'y', label='INSPVAX')

plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('position accuracy(m)')

# plt.plot(lat_std)
# plt.plot(lng_std)
# plt.plot(alt_std)
plt.title("the Accuracy of Novatel GPS 2d Position")
print "mean of 2d position accuracy: ", np.mean(pos_2d_std)

plt.show()


