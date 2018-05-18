import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np


bagname = sys.argv[1]
bag = rosbag.Bag(bagname)
ins_t = []

for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    pc_time = float(str(msg.header2.stamp.secs)) \
        + float(str(msg.header2.stamp.nsecs)) * \
        1e-9  # epoch second
    novatel_time = 315964800 + msg.header.gps_week * 7 * 24 * 60 * 60 \
            + float(msg.header.gps_week_seconds) / \
            1000 - 18  # GPS time to epoch second    
    ins_t.append(novatel_time)

ins_t = np.array(ins_t)
diff = ins_t[1:] - ins_t[:-1]
plt.plot(diff, 'r', label='time space between 2 stamps')
plt.show()
