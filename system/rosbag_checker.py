#!/usr/bin/env python

import rosbag
import rospy
import sys
import matplotlib.pyplot as plt


class RosbagChecker(object):

    def __init__(self, bag):
        self.bag = bag
        self.time_diff = []

    def check_bag(self):
        bag = rosbag.Bag(self.bag)
        time_diff = self.check_gps_sync(bag)
        print 'time diff is: ', time_diff
        if time_diff < 0.08:
            print 'sync is ok'
        else:
            print 'sync failed'

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
            self.time_diff.append(novatel_time - pc_time)
            sum_abs_diff += abs(novatel_time - pc_time)
            count += 1
        if count == 0:
            return 0
        avg_diff = sum_abs_diff / count
        rospy.loginfo(
            '[rosbag_recorder] check gps & pc time diff: ' + str(avg_diff) + 's.')
        # Sync if average difference less than 80ms
        return avg_diff

    def plot(self):
        plt.plot(self.time_diff, 'r.', label='time diff between pc and gps')
        plt.ylabel('t/s')
        plt.legend(loc='upper left')
        plt.show()

if __name__ == '__main__':
    bagname = sys.argv[1]
    checker = RosbagChecker(bagname)
    checker.check_bag()
    checker.plot()
