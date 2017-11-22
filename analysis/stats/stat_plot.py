# Stats & Difference Plot
# Author: Jinhan Zhang
# Date: 2017/07/17

import matplotlib.pyplot as plt
import sys
import math
import numpy as np
import rosbag

# Constant
EARTH_RADIUS = 6378137.0
topic_name = ''

# Change the following constant
NO_RESULT_THRE = 1000
SAMPLE_NUM = 1

class Orientation:
    def __init__(self):
        #define file path
        bag_path = './'
        bagname = bag_path + sys.argv[1]
        self.bag = rosbag.Bag(bagname)

        #Constant
        self.baseLat = self.DEG2RAD(32.694052)
        self.baseLon = self.DEG2RAD(-113.958389)

        #Initialize
        self.latitude_GPS, self.longitude_GPS, self.latitude_Nov, self.longitude_Nov = [], [], [], []
        self.latitude_GPS_raw, self.longitude_GPS_raw, self.latitude_Nov_raw, self.longitude_Nov_raw = [], [], [], []

        #Keep track of time
        self.secs_GPS, self.nsecs_GPS, self.secs_Nov, self.nsecs_Nov = [], [], [], []
    	self.x1, self.y1, self.x2, self.y2 = [], [], [], []

    	self.diff_x, self.diff_y = np.array([]), np.array([])

if __name__ == '__main__':
    ori = Orientation()
    ori.readbag()
    ori.calxy()
    ori.plot()
