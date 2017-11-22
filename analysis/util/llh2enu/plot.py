from llh2enu_gps_transformer import *
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import sys

class GPSPlot():
    def __init__(self):
        bag_path = '/home/bricy/workspace/rosbag/'
        # bagname = bag_path + '_2017-09-24-17-22-14_6.bag'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)
        self.x1 = []
        self.y1 = []
        self.x2 = []
        self.y2 = []
        self.x5 = []
        self.y5 = []
        self.x3 = []
        self.y3 = []
        self.first_flag = False
        self.lat0 = 0
        self.lon0 = 0

    def read_bag(self):
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/bestpos']):
            test = gps_transformer()
            if not self.first_flag:
                self.lat0 = msg.latitude
                self.lon0 = msg.longitude
                self.first_flag = True
                # self.lat0 = 39.714178
                # self.lon0 = 117.305466

                # self.lat0 = 39.03775082210
                # self.lon0 = 118.43091220755
                # self.lat0 = self.lat0 * np.pi / 180
                # self.lon0 = self.lon0 * np.pi / 180
                x1_0, y1_0 = test.llh2enu_1(self.lat0, self.lon0, 0, self.lat0, self.lon0, 0)
                x2_0, y2_0 = test.llh2enu_2(self.lat0, self.lon0, 0, self.lat0, self.lon0, 0)
                x3_0, y3_0 = test.llh2enu_3(self.lat0, self.lon0, self.lat0, self.lon0)
                x5_0, y5_0 = test.llh2enu5(self.lat0, self.lon0, self.lat0, self.lon0)
            else:
                lat = msg.latitude
                lon = msg.longitude
                x1, y1 = test.llh2enu_1(lat, lon, 0, self.lat0, self.lon0, 0)
                x5, y5 = test.llh2enu5(lat, lon, self.lat0, self.lon0)
                x2, y2 = test.llh2enu_2(lat, lon, 0, self.lat0, self.lon0, 0)
                x3, y3 = test.llh2enu_3(lat, lon, self.lat0, self.lon0)
                # method referenced http://digext6.defence.gov.au/dspace/bitstream/1947/3538/1/DSTO-TN-0432.pdf
                if len(self.x1) > 0:
                    self.x1.append(x1 - self.x1[0])
                    self.y1.append(y1 - self.y1[0])
                    # from wiki
                    self.x2.append(x2 - self.x2[0])
                    self.y2.append(y2 - self.y2[0])

                    # from kitti
                    self.x3.append((x3 - x3_0) - self.x3[0])
                    self.y3.append((y3 - y3_0) - self.y3[0])

                    # octopus
                    self.x5.append((x5 - x5_0) - self.x5[0])
                    self.y5.append((y5 - y5_0) - self.y5[0])
                else:
                    self.x1.append(x1)
                    self.y1.append(y1)
                    self.x2.append(x2)
                    self.y2.append(y2)
                    self.x3.append(x3 - x3_0)
                    self.y3.append(y3 - y3_0)
                    self.x5.append(x5 - x5_0)
                    self.y5.append(y5 - y5_0)

        self.x1[0] = 0
        self.y1[0] = 0
        self.x2[0] = 0
        self.y2[0] = 0
        self.x3[0] = 0
        self.y3[0] = 0
        self.x5[0] = 0
        self.y5[0] = 0


    def plot(self):
        plt.subplot(411)
        plt.plot(self.x1, self.y1, 'b', label='defence')
        plt.plot(self.x2, self.y2, 'y', label='wiki')
        plt.plot(self.x3, self.y3, 'g', label='kitti')
        plt.plot(self.x5, self.y5, 'r', label='octopus')
        plt.legend(loc='upper right')
        plt.subplot(412)
        plt.plot(self.x2, self.y2, 'b', label='wiki')
        plt.legend(loc='upper left')
        plt.subplot(413)
        plt.plot(self.x3, self.y3, 'b', label='kitti')
        plt.legend(loc='upper left')
        plt.subplot(414)
        plt.plot(self.x5, self.y5, 'r', label='octopus')
        plt.legend(loc='upper left')

        plt.show()

    # return the distance between (x,y) and (x0,y0)
    def dist(self, x, y, x0, y0):
        return np.sqrt((x-x0)**2 + (y-y0)**2)

    def run(self):
        self.read_bag()
        self.plot()

if __name__ == '__main__':
    p = GPSPlot()
    p.run()
