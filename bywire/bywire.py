## Plot the Novatel BESTPOS's position type and position accuracy together
import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np


class PosType:
    def __init__(self):
        bag_path = '../../rosbag/'
        self.bagname = []
        self.bagname.append(bag_path + sys.argv[1])
        if len(sys.argv) >= 3:
            self.bagname.append(bag_path + sys.argv[2])
        self.bag = []
        for bag in self.bagname:
            self.bag.append(rosbag.Bag(bag))
        self.pedal_input = []
        self.pedal_output = []
        self.enable = []
        self.override = []
        self.pedal_cmd = []
        self.driver = []
        self.speed = []
        self.index = [0, 0]

    def readbag(self):
        count = 0
        for bag in self.bag:
            for topic, msg, t in bag.read_messages(topics=['/vehicle/throttle_report']):
                self.pedal_input.append(msg.pedal_input)
                self.pedal_output.append(msg.pedal_output)
                self.enable.append(msg.enabled)
                self.override.append(msg.override*0.3)
                self.pedal_cmd.append(msg.pedal_cmd)
                self.driver.append(msg.driver)
            for topic, msg, t in bag.read_messages(topics=['/vehicle/twist']):
                self.speed.append(msg.twist.linear.x/25)
            count += 1


            bag.close()

    def plot(self):
        plt.subplot(511)
        plt.plot(self.pedal_input, 'r', label='pedal input')
        plt.plot(self.override, 'g', label='override')
        plt.plot(self.speed, 'b', label='speed')
        plt.legend(loc='upper left')

        plt.title("Pedal Input")
        plt.subplot(512)
        plt.plot(self.pedal_output, 'r')
        plt.title("Pedal Output")
        plt.subplot(513)
        plt.plot(self.enable, 'r')
        # plt.plot(self.driver, 'g')

        plt.ylim([0, 2])
        plt.title("Enable")
        plt.subplot(514)
        plt.plot(self.override, 'r')
        plt.ylim([0, 2])
        plt.title("Override")
        plt.subplot(515)
        plt.plot(self.pedal_cmd, 'r')


        # plt.legend(loc='upper left')
        # plt.xlabel('stamp')
        # plt.ylabel('degree')
        # plt.title("Heading accuracy ")
        plt.show()

if __name__ == '__main__':
    pos = PosType()
    pos.readbag()
    print 'min of pedal input: ', round(np.min(pos.pedal_input),4)
    pos.plot()

