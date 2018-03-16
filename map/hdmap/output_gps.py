# Plot the Novatel BESTPOS's position type and position accuracy together
import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np
import glob


class PosType:
    def __init__(self):
        bag_path = "/home/zhibei/workspace/rosbag/2018-03-16/"
        self.file_name = 'gps_' + bag_path.split('/')[-2] + '.txt'
        self.bagname = glob.glob(bag_path + "*.bag")
        print 'bagname: ', self.bagname
        self.bag = []
        for bag in self.bagname:
            print 'bbb'
            self.bag.append(rosbag.Bag(bag))

    def readbag(self):
        with open(self.file_name, 'a') as the_file:
            for bag in self.bag:
                for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                    print msg.latitude
                    the_file.write(str(msg.latitude) + ' ' +
                                   str(msg.longitude) + ' ' + str(msg.azimuth) + ' \n')
                bag.close()


if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
