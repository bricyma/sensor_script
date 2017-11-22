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

    def plot(self):
        plt.subplot(411)
        plt.plot(self.x1, 'r', label='Test GPS')
        plt.plot(self.x2, 'b', label='Novatel')
        plt.legend(loc='upper left')
        plt.title('X-axis')

        plt.subplot(412)
        plt.plot(self.y1, 'r', label='Test GPS')
        plt.plot(self.y2, 'b', label='Novatel')
        plt.legend(loc='upper left')
        plt.title('Y-axis')

        plt.subplot(413)
        plt.plot(self.diff_x, 'r', label='xdiff')
        plt.legend(loc='upper left')
        plt.title('X-diff')

        plt.subplot(414)
        plt.plot(self.diff_y, 'r', label='ydiff')
        plt.legend(loc='upper left')
        plt.title('Y-diff')
        plt.show()

    def DEG2RAD(self, x):
        return x / (180 / math.pi)

    def RAD2DEG(self, x):
        return x * (180 / math.pi)

    def latlon2xy(self, lat_, lon_):
        lat, lon = self.DEG2RAD(lat_), self.DEG2RAD(lon_)
        xx = math.cos(lat) * math.cos(lon) * math.cos(self.baseLon) * math.cos(self.baseLat) \
            + math.cos(lat) * math.sin(lon) * math.sin(self.baseLon) * math.cos(self.baseLat) \
            + math.sin(lat) * math.sin(self.baseLat)
        yy = -math.cos(lat) * math.cos(lon) * math.sin(self.baseLon) \
            + math.cos(lat) * math.sin(lon) * math.cos(self.baseLon)
        zz = -math.cos(lat) * math.cos(lon) * math.cos(self.baseLon) * math.sin(self.baseLat) \
             - math.cos(lat) * math.sin(lon) * math.sin(self.baseLon) * math.sin(self.baseLat) \
            + math.sin(lat) * math.cos(self.baseLat)
        x = math.atan2(yy, xx) * EARTH_RADIUS
        y = math.log(math.tan(math.asin(zz) / 2 + math.pi / 4)) * EARTH_RADIUS

        return x, y

    def readbag(self):
        cnt = 0
	topic_name = sys.argv[2]
        #Read bag from Swift
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            if cnt % SAMPLE_NUM == 0:
                self.latitude_GPS_raw.append(msg.latitude)
                self.longitude_GPS_raw.append(msg.longitude)
                self.secs_GPS.append(msg.header2.stamp.secs)
                self.nsecs_GPS.append(msg.header2.stamp.nsecs)
            cnt += 1

        #Read bag from Novatel
        for topic, msg, t in self.bag.read_messages(topics=[topic_name]):
            self.latitude_Nov_raw.append(msg.latitude)
            self.longitude_Nov_raw.append(msg.longitude)
            self.secs_Nov.append(msg.header.stamp.secs)
            self.nsecs_Nov.append(msg.header.stamp.nsecs)
        
        self.bag.close()

        len_GPS, len_Nov = len(self.latitude_GPS_raw), len(self.latitude_Nov_raw)
        print " length of Novatel data: ", len_GPS, "length of tested GPS data: ", len_Nov

        j = 0
        for i in range(len_GPS):
            while(j < len_Nov-1):
            	j += 1
            	#print i,j
                if((self.secs_Nov[j] == self.secs_GPS[i] and self.nsecs_Nov[j] > self.nsecs_GPS[i]) or self.secs_Nov[j] > self.secs_GPS[i]):
                    #print i,j
                    self.latitude_GPS.append(self.latitude_GPS_raw[i])
                    self.longitude_GPS.append(self.longitude_GPS_raw[i])
                    self.latitude_Nov.append(self.latitude_Nov_raw[j])
                    self.longitude_Nov.append(self.longitude_Nov_raw[j])
                    break;

        #print "length of Swift data: ", len(self.latitude_Nov), " length of Novatel data: ", len(self.latitude_GPS)

        if len(self.latitude_GPS) != len(self.latitude_Nov):
            raise NameError('There is no 1-to-1 match! Please make sure the amount of data obtained by new GPS is less than Novatel...')
        print "The number of matched data points: ", len(self.longitude_Nov)

    def calxy(self):
        size = len(self.longitude_Nov)
        for i in range(size):
            #1
            x_temp1, y_temp1 = self.latlon2xy(self.latitude_GPS[i],self.longitude_GPS[i])
            #2
            x_temp2, y_temp2 = self.latlon2xy(self.latitude_Nov[i],self.longitude_Nov[i])

	    if (abs(x_temp1-x_temp2) < NO_RESULT_THRE and abs(y_temp1-y_temp2) < NO_RESULT_THRE) :
    		self.x1.append(x_temp1)
    		self.y1.append(y_temp1)
    		self.x2.append(x_temp2)
    		self.y2.append(y_temp2)

        size = len(self.x1)
    	self.diff_x = np.array([self.x2[i] - self.x1[i] for i in range(size)])
    	self.diff_y = np.array([self.y2[i] - self.y1[i] for i in range(size)])
        print ""
        print "Stats:"

    	print "diff_Mean_x: ", np.mean(self.diff_x), "diff_Mean_y: ", np.mean(self.diff_y)
    	print "diff_Std_x: ", np.std(self.diff_x), "diff_Std_y: ", np.std(self.diff_y)

        diff_x_max = max(np.max(self.diff_x),-np.min(self.diff_x))
        diff_y_max = max(np.max(self.diff_y),-np.min(self.diff_y))
    	print "diff_Max_x: ", diff_x_max, "diff_Max_y: ", diff_y_max

        res_sqr = 0
        for i in range(size):
            res_sqr += math.sqrt(self.diff_x[i]**2 + self.diff_y[i]**2)
        dist_avg_xy = res_sqr / size
        print "dist_avg_xy: ", dist_avg_xy

if __name__ == '__main__':
    ori = Orientation()
    ori.readbag()
    ori.calxy()
    ori.plot()
