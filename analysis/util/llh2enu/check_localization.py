from llh2enu_gps_transformer import *
import matplotlib.pyplot as plt
import rosbag
import numpy as np
import sys

class GPSPlot():
    def __init__(self):
        self.bag_path = '../../testbag/'
        # bagname = bag_path + '_2017-09-11-16-41-43_3.bag'
        # self.bagname = ['_2017-09-11-15-29-01_3.bag',  '_2017-09-11-15-44-01_6.bag',  '_2017-09-11-15-59-01_9.bag',
                        # '_2017-09-11-15-34-01_4.bag',  '_2017-09-11-15-49-01_7.bag',  '_2017-09-11-16-04-01_10.bag',
                        # '_2017-09-11-15-39-01_5.bag',  '_2017-09-11-15-54-01_8.bag', '_2017-09-11-16-09-01_11.bag']
        # bagname = bag_path + sys.argv[1]
        self.bagname = ['_2017-09-11-16-41-43_3.bag']
        # self.bag = rosbag.Bag(bagname)



        self.gnss_pose_x = []
        self.gnss_pose_y = []
        self.gnss_pose_t = []
        self.image_pose_x = []
        self.image_pose_y = []
        self.image_pose_t = []
        self.diff_x = []
        self.diff_y = []
        self.diff = []  # 2d distance
        self.gnss_id = [] # the index of gnss/pose compare with image_localization
        self.compare_gnss_pose_x = []
        self.compare_gnss_pose_y = []

    def iterate_bags(self):
        count = 0
        for bagname in self.bagname:
            bag = rosbag.Bag(self.bag_path + bagname)
            self.read_bag(bag)
            count += 1


    def read_bag(self, bag):
        for topic, msg, t in bag.read_messages(topics=['/image_localization_fusion/pose']):
            self.image_pose_x.append(msg.pose.position.x)
            self.image_pose_y.append(msg.pose.position.y)
            self.image_pose_t.append(msg.header.stamp.to_sec())

        for topic, msg, t in bag.read_messages(topics=['/localization/pose']):
            self.gnss_pose_x.append(msg.pose.position.x)
            self.gnss_pose_y.append(msg.pose.position.y)
            self.gnss_pose_t.append(msg.header.stamp.to_sec())

    def analysis(self):
        start = 0
        delay = 0
        count = 0
        for t in self.image_pose_t:
            if count < 100:
                count += 1
                self.diff_x.append(0)
                self.diff_y.append(0)
                continue
            i = start
            while i < len(self.gnss_pose_t)-1:
                left = self.gnss_pose_t[i]
                right = self.gnss_pose_t[i+1]
                if self.gnss_pose_t[i] + delay <= t <= self.gnss_pose_t[i+1] + delay:
                    if t - self.gnss_pose_t[i] > self.gnss_pose_t[i+1] - t:
                        self.gnss_id.append(i+1)
                    else:
                        self.gnss_id.append(i)
                    break
                else:
                    i += 1
            start = i
        if len(self.image_pose_t) > len(self.gnss_id):
            self.gnss_id.append(len(self.gnss_pose_t)-1)
        # get the gnss_pose_t with gnss_id
        i = 0
        for index in self.gnss_id:
            # print self.image_pose_t[i], self.gnss_pose_t[index]
            x0 = self.gnss_pose_x[index]
            y0 = self.gnss_pose_y[index]
            x = self.image_pose_x[i+100]
            y = self.image_pose_y[i+100]
            self.diff.append(self.dist(x0, y0, x, y))
            self.diff_x.append(abs(x0-x))
            self.diff_y.append(abs(y0-y))
            i += 1
            self.compare_gnss_pose_x.append(x0)
            self.compare_gnss_pose_y.append(y0)

        for diff_xx in self.diff_x:
            print diff_xx

    def distance(self, x, y, x0, y0):
        return np.sqrt((x-x0)**2 + (y-y0)**2)


    def plot(self):
        plt.subplot(411)
        plt.plot(self.image_pose_x, self.image_pose_y, 'b', label='image localization')
        plt.legend(loc='upper left')
        plt.subplot(412)
        plt.plot(self.gnss_pose_x, self.gnss_pose_y, 'b', label='gnss')
        plt.legend(loc='upper left')
        plt.subplot(413)
        plt.plot(self.diff_x, 'b', label='diff x')
        plt.plot(self.diff_y, 'r', label='diff y')
        plt.plot(self.diff, 'g', label='diff')
        plt.legend(loc='upper left')
        plt.subplot(414)
        plt.plot(self.image_pose_x, self.image_pose_y, 'b', label='image localization')
        plt.plot(self.gnss_pose_x, self.gnss_pose_y, 'r', label='gnss')
        plt.legend(loc='upper left')
        plt.show()

    # return the distance between (x,y) and (x0,y0)
    def dist(self, x, y, x0, y0):
        return np.sqrt((x-x0)**2 + (y-y0)**2)

    def run(self):
        self.iterate_bags()
        self.analysis()
        self.plot()

if __name__ == '__main__':
    p = GPSPlot()
    p.run()
