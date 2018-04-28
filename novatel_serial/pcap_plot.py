#! /usr/bin/env python

import dpkt
import matplotlib.pyplot as plt
import numpy as np
import sys

file_name = sys.argv[1]
f = open(file_name)
pcap = dpkt.pcap.Reader(f)
capture_timestamp = []
for timestamp, buf in pcap:
    eth = dpkt.ethernet.Ethernet(buf)
    ip = eth.data
    tcp = ip.data
    try:
        if ip.len == 210:  # inspvax: 210; inspvas: 124
            capture_timestamp.append(timestamp)
    except:
        pass
capture_timestamp = np.array(capture_timestamp)

interval = capture_timestamp[1:] - capture_timestamp[:-1]

font = {'family': 'normal', 'weight': 'bold', 'size': 30}
plt.rc('font', **font)
# plt.plot(interval, 'r', label='inspvax interval')
plt.plot(interval, 'r', label='inspvax interval')
plt.xlabel('stamp')
plt.ylabel('time(s)')

plt.legend(loc='upper left')
plt.show()
