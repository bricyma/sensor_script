#!/usr/bin/env python

import os
import rosbag
import rospy
import sys
import matplotlib.pyplot as plt
import numpy as np

class RosbagChecker(object):

    def __init__(self, name):
        self.time_diff = []
        bag = rosbag.Bag(name)

        self.rdata = {'t': []}
        self.check_gps_sync(bag)

    def check_gps_sync(self, bag):
        # Check if gps time is sync with pc time in a specific bag.
        sum_abs_diff = 0
        count = 0
        for data in bag.read_messages(topics=['/novatel_data/inspvax']):
            novatel_time = 315964800 + data[1].header.gps_week * 7 * 24 * 60 * 60 \
                + float(data[1].header.gps_week_seconds) / \
                1000 - 18  # GPS time to epoch second
            pc_time = float(str(data[1].header2.stamp.secs)) \
                + float(str(data[1].header2.stamp.nsecs)) * \
                1e-9  # epoch second
            self.rdata['t'].append(pc_time)
            self.time_diff.append(novatel_time - pc_time)
            sum_abs_diff += abs(novatel_time - pc_time)
            count += 1
        if count == 0:
            return 0
        avg_diff = sum_abs_diff / count
        rospy.loginfo(
            '[rosbag_recorder] check gps & pc time diff: ' + str(avg_diff) + 's.')
        # Sync if average difference less than 80ms
        print 'time diff is: ', avg_diff
        if avg_diff < 0.135:
            print '\033[32msync is ok'
        else:
            print '\033[31msync failed'

    def plot(self):
        # plt.plot(self.time_diff, 'r.', label='time diff between pc and gps')
        print np.mean(np.array(
            self.rdata['t'][1:]) - np.array(self.rdata['t'][:-1]))
        plt.plot(np.array(
            self.rdata['t'][1:]) - np.array(self.rdata['t'][:-1]), 'r', label='time diff')
        plt.ylabel('t/s')
        plt.legend(loc='upper left')
        plt.show()


if __name__ == '__main__':
    name = sys.argv[1]
    checker = RosbagChecker(name)
    checker.plot()
