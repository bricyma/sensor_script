# compare imu result in different positions
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
import sys


bag_name = '2018-04-30-18-05-44'
ts_begin = '0:59'
ts_end = '30:01'
info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
parser = DataParser(info)
pkt_left, pkt_right = parser.parse_pandar()

pkts_len_left, pkts_len_right, t = [], [], []
for pkt in pkt_left:
    pc_time = float(str(pkt.header.stamp.secs)) \
        + float(str(pkt.header.stamp.nsecs)) * \
        1e-9  # epoch second
    t.append(pc_time)
    pkts_len_left.append(len(pkt.packets))
t = np.array(t)
# for pkt in pkt_right:
        # pkts_len_right.append(len(pkt.packets))
# def plot()
font = {'family': 'normal',
        'weight': 'bold',
        'size': 20}
plt.rc('font', **font)
# accelration
fig = plt.figure(1)
fig.suptitle('Acceleration in 3 axle', fontsize=30)

plt.subplot(211)
plt.ylim(0, 1)
diff = t[1:] - t[:-1]
print diff
# plt.set_ylim(ymin=0.0, ymax=1)
plt.plot(diff, 'r.', label='time diff')
print diff[0], diff[1]
plt.subplot(212)
plt.plot(pkts_len_left, 'r', label='left pandar packet length')
# plt.plot(pkts_len_right, 'b', label='right pandar packet length')
plt.xlabel('stamp')
plt.show()
