# author: Zhibei Ma
# date: 2018/04/26
# check timstamp is stable


import rosbag
import sys
import matplotlib.pyplot as plt
import numpy as np
BAG_PATH = ''

class GnssImuParser:
    def __init__(self):
        self.bag_list = []
        self.bag_list.append(sys.argv[1])  # for only one bag analysis
        for i in range(0, len(self.bag_list)):
            self.bag_list[i] = BAG_PATH + self.bag_list[i]
        # data init
        self.data = {'insspd': [], 'bestvel': [], 'inspvax': []}
        self.readbag()
        self.plot()

    def readbag(self):
        for bag in self.bag_list:
            bag = rosbag.Bag(bag)
            count = 0
            # parse novatel
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
                self.data['inspvax'].append(msg.azimuth)
            # parse insspd
            for topic, msg, t in bag.read_messages(topics=['/novatel_data/insspd']):
                self.data['insspd'].append(msg.track_ground)

        ins_len = len(self.data['inspvax'])
        self.data['inspvax'] = np.array(self.data['inspvax'])
        self.data['insspd'] = np.array(self.data['insspd'][0:ins_len])

    def plot(self):
        plt.subplot(211)
        plt.plot(self.data['inspvax'], 'r', label='inspvax')
        plt.plot(self.data['insspd'], 'b', label='insspd')
        plt.legend(loc='upper left')
        plt.subplot(212)
        diff = self.data['inspvax'] - self.data['insspd']
        plt.plot(diff, 'r', label='inspvax - insspd')
        plt.legend(loc='upper left')  
        print 'mean of (inspvax-insspd): ', np.mean(diff)      
        plt.show()


if __name__ == '__main__':
    _ = GnssImuParser()