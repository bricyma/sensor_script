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
diff = []

for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    gps_time = 315964800 + 7 * msg.header.gps_week * 60 * 60 * 24 + float(msg.header.gps_week_seconds)/1000 - 18
    print int(gps_time), gps_time - int(gps_time)

    gps_time_sec.append(gps_time)
    ecu_time_sec.append(msg.header2.stamp.secs)
    diff.append(gps_time - msg.header2.stamp.to_sec())

bag.close()
# plt.plot(gps_time_sec, 'b', label='gps')
# plt.plot(ecu_time_sec, 'g', label='ecu')
plt.plot(diff, 'r', label='diff')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.title("Time between GPS and ECU")
plt.show()



