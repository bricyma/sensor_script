# Plot the Novatel BESTPOS's position type and position accuracy together
import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np
import glob


class PosType:
    def __init__(self):
        bag_path = sys.argv[1] + '/'
        # bag_path = "/home/bricy/workspace/rosbag/hdmap/2018-03-24-17-52-22/"
        self.file_name = 'gps_' + bag_path.split('/')[-2] + '.txt'
        self.bagname = sorted(glob.glob(bag_path + "*.bag"))
        print 'bagname: ', self.bagname
        self.bag = []
        self.bestvel = []  # azimuth from topic /novatel_data/bestvel
        self.insspd = [] # 
        self.dis_to_lane_center = []
        for bag in self.bagname:
            print 'bbb'
            self.bag.append(rosbag.Bag(bag))

    def readbag(self):
        for bag in self.bag:
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/bestvel']):
                self.bestvel.append(msg.trk_gnd)

            for topic, msg, t in bag.read_messages(topics=['/novatel_data/insspd']):
                self.insspd.append(msg.track_ground)
            # for topic, msg, t in bag.read_messages(topics=['/error_monitor/error_info']):
            #     self.dis_to_lane_center.append(msg.dis_to_lane_center)
        with open(self.file_name, 'w') as the_file:
            count = 0
            for bag in self.bag:
                for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                    the_file.write(str(msg.latitude) + ' ' +
                                   str(msg.longitude) + ' ' +
                                   str(msg.azimuth) + ' ' +
                                   str(msg.roll) + ' ' + 
                                   str(msg.pitch) + ' ' + 
                                   str(self.bestvel[int(count / 2.5)]) + ' ' +
                                   str(self.insspd[count])  + ' \n') 
                    count += 1
                    # print count
                    if count >= len(self.bestvel) * 2.5:
                        break
                bag.close()


if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
