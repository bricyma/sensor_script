## Plot the Novatel BESTPOS's position type and position accuracy together
import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np


class PosType:
    def __init__(self):
        bag_path = '../../rosbag/'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.yaw = []  # bag1 position accuracy
        self.position_type = []
        self.ins_status = []
        self.yaw_accuracy = []

    def readbag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            self.yaw.append(msg.azimuth)
            self.position_type.append(msg.position_type)
            self.ins_status.append(msg.ins_status)
            self.yaw_accuracy.append(msg.azimuth_std)
        self.bag.close()

    def plot(self):
        plt.subplot(511)
        plt.plot(self.yaw, 'r')
        plt.plot(self.position_type, 'b')
        plt.title("Yaw")

        plt.subplot(512)
        plt.plot(self.ins_status, 'r')
        plt.title('ins status')

        plt.subplot(513)
        plt.plot(self.yaw_accuracy, 'r')
        plt.title('yaw accuracy')

        plt.show()

if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
    pos.plot()
