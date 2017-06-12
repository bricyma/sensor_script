## Plot the Novatel BESTPOS's position type and position accuracy together
import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np


class PosType:
    def __init__(self):
        bagname = '../../rosbag/'
        bagname += sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.postype = []
        self.posacc = [] # position accuracy
        self.satellite_used = [] # satellites used in solution
        self.satellite = []

    def readbag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/bestpos']):
            self.posacc.append(m.sqrt(msg.latitude_std ** 2 + msg.longitude_std ** 2))
            self.postype.append(msg.position_type)
            self.satellite.append(msg.sol_svs)
        self.bag.close()

    def plot(self):
        plt.subplot(311)
        plt.plot(self.postype, 'r')
        plt.title("Position Type")

        plt.subplot(312)
        plt.plot(self.posacc, 'r')
        plt.title("Position Accuracy")

        plt.subplot(313)
        plt.plot(self.satellite, 'r')
        plt.title("Satellites used in solution")

        plt.show()

if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
    pos.plot()
    print np.mean(pos.satellite)

