#!/usr/local/bin/python2.7

import dpkt
import sys
import matplotlib.pyplot as plt
import numpy as np
import datetime
counter = 0
ipcounter = 0
tcpcounter = 0
udpcounter = 0

filename = sys.argv[1]

data = {}

for ts, pkt in dpkt.pcap.Reader(open(filename, 'r')):

    # print 'Timestamp: ', str(datetime.datetime.utcfromtimestamp(ts))
    counter += 1
    eth = dpkt.ethernet.Ethernet(pkt)
    if eth.type != dpkt.ethernet.ETH_TYPE_IP:
        continue

    ip = eth.data
    ipcounter += 1

    if ip.len in data:
        data[ip.len].append(float(ts))
    else:
        data[ip.len] = []

    if ip.p == dpkt.ip.IP_PROTO_TCP:
        tcpcounter += 1

    if ip.p == dpkt.ip.IP_PROTO_UDP:
        udpcounter += 1

for k in data:
    if len(data[k]) < 1000:
        continue
    plt.plot(np.array(data[k][1:]) - np.array(data[k][:-1]), 'r', label=str(k))
    plt.legend(loc='upper left')
    plt.show()
print "Total number of packets in the pcap file: ", counter
print "Total number of ip packets: ", ipcounter
print "Total number of tcp packets: ", tcpcounter
print "Total number of udp packets: ", udpcounter
