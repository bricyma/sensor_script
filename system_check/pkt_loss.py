import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np

bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)

before = 0
count = 0
time_start = 0
time_end = 0
period = 20
for topic, msg, t in bag.read_messages(topics=['/novatel_data/rawimu']):
    if time_start == 0:
        time_start = msg.header.gps_week_seconds
    time_end = msg.header.gps_week_seconds
    count += 1
    time1 = msg.header.gps_week_seconds
    now = time1
    if now - before != period:
        print now-before
    before = now

print "total seq: ", count
print "expected seq: ", (time_end - time_start) / period + 1
print "loss packet: ", (time_end - time_start) / period + 1 - count
# print "loss rate: ", float(loss_num)/sequence

