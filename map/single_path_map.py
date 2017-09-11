#!/usr/bin/env python
from vsimple.map import Map
import ujson as json
import time
import rosbag

class DrawOnMap():
    def __init__(self):
        bag_path = '../../rosbag/'
        bagname = bag_path + '_2017-08-25-12-39-12_6.bag'
        # bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.novatel_lat = []
        self.novatel_lng = []

    def read_bag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/bestpos']):
            self.novatel_lat.append(msg.latitude)
            self.novatel_lng.append(msg.longitude)

    def callback(self):
        m1 = Map()
        m1.go(self.novatel_lat[0], self.novatel_lng[0], 15)

        for i in xrange(0, len(self.novatel_lat)/10, 1):
            print self.novatel_lat[10*i]
            m1.scatter(self.novatel_lat[10*i], self.novatel_lng[10*i], 'green')
            time.sleep(0.005)


if __name__ == '__main__':
    m = DrawOnMap()
    m.read_bag()
    m.callback()


