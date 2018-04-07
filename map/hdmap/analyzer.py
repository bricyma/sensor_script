#!/usr/bin/env python

# usage: python yaw_analyzer.py
# YawAnalyzer is used to get the relationship between azimuth from INSPVAX, 
# trk_gnd from BESTVEL and local yaw from coordinate transformation.
# input: 
# gps file containing latitude, longitude, azimuth, trk_gnd
# hdmap
# output: figure is stored in folder figure
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy, xy2latlon
import sys
from yaw_conversion1 import *
from collections import deque
import time
import glob
from llh2enu.llh2enu_gps_transformer import *
from gnss_transformer import GPSTransformer

class YawAnalyzer:
    def __init__(self, error_data, gps_data):
        start_time = time.time()
        # map_file = "I-10_Phoenix2Tucson_20180315.hdmap"
        map_file = "I-10_Tucson2Phoenix_20180327.hdmap"
        # yaw is derived from x,y path in global coordination
        # yaw2 is derived from Yaw conversion by calculating the offset between north and true north
        # azimuth is from azimuth in INSPVAX
        self.gps_data = gps_data
        self.error_data = error_data
        self.data = {'lat': [], 'lon': [], 'x': [], 'y': [], 'azimuth': [], 'trk_gnd': [], 'yaw': [], 'yaw2': []}
        self.yaw_conver = YawConversion()
        self.transformer = gps_transformer() 
        self.vehicle_gnss_trans = GNSSTransformer() # octopus's llh2enu transformation

        with open(map_file, 'rb') as f:
            submap = f.read()
        self.map = TSMap(submap)
        self.parse_yaw()


    def parse(self):
        # yaw1: inspvax - local_yaw
        # yaw2: bestvel - local_yaw
        # yaw3: inspvax - bestvel
        data_no_window = {'d_yaw1': [], 'd_yaw2': [], 'd_yaw3': []}
        data_window = {'d_yaw1': [], 'd_yaw2': [], 'd_yaw3': []}
        window_size = 1000
        for k in data_window:
            if k == 'd_yaw1':
                data_window[k] = self.parse_window(self.data['azimuth'] - self.data['yaw2'], window_size)
                data_no_window[k] = self.parse_window(self.data['azimuth'] - self.data['yaw2'], 1)
            elif k == 'd_yaw2':
                data_window[k] = self.parse_window(self.data['trk_gnd'] - self.data['yaw2'], window_size)
                data_no_window[k] = self.parse_window(self.data['trk_gnd'] - self.data['yaw2'], 1)
            else:
                data_window[k] = self.parse_window(self.data['azimuth'] - self.data['trk_gnd'], window_size)
                data_no_window[k] = self.parse_window(self.data['azimuth'] - self.data['trk_gnd'], 1)

        return self.parse_window(self.error_data, window_size), self.parse_window(self.error_data, 1), \
                data_window, data_no_window


    # return windowed data
    def parse_window(self, data, window_size):
        sum = 0
        diff2 = [0]
        # threshold: 1
        # filter those data >= 1
        data_window = []
        for i, d in enumerate(data):
            if 359 < d <  361:
                d -= 360
            if abs(d) < 1:
                diff2.append(d)
            elif d > 1:
                # diff2.append(diff2[i-1])
                diff2.append(0)
            else:
                diff2.append(0)
        for i in range(0, len(diff2)):
            sum += diff2[i]
            if i >= window_size:
                sum -= diff2[i-window_size]
                ave = sum/window_size
            else:
                ave = sum/(i+1)
            data_window.append(ave)
        return data_window

    # get local yaw through coordinate transformation
    def parse_yaw(self):
        for i in range(0, len(self.gps_data['lat'])):
            try:
                lat, lon, azimuth, trk_gnd = self.gps_data['lat'][i], self.gps_data['lon'][i], \
                                            self.gps_data['azimuth'][i], self.gps_data['trk_gnd'][i],  
                x, y = latlon2xy(lat, lon)
                p = Point3d(x, y)   
                # ref_p = self.map.get_ref_pt(p)  # filter those not in map
                self.data['lat'].append(lat)
                self.data['lon'].append(lon)
                self.data['azimuth'].append(azimuth)
                self.data['trk_gnd'].append(trk_gnd)
                self.data['x'].append(x)                    
                self.data['y'].append(y)
                # print self.data
            except:
                pass
        # global to local, x,y,x2,y2 => lat,lng => x',y', x2',y2' => yaw2
        # for i in range(0, len(self.data['x'])-1):
        #     x = self.data['x'][i]
        #     y = self.data['y'][i]
        #     x2 = self.data['x'][i+1]
        #     y2 = self.data['y'][i+1]
        #     yaw = self.yaw_conver.pos_conversion3(x,y,x2,y2)
        #     if yaw < -10:
        #         yaw += 360
        #     self.data['yaw2'].append(yaw)

        # another llh2enu method
        base_lat = 32.75707  # center between tucson and phoenix
        base_lon = -111.55757    
        for i in range(0, len(self.data['lat'])-1):
            lat = self.data['lat'][i]
            lon = self.data['lon'][i]        
            lat2 = self.data['lat'][i+1]
            lon2 = self.data['lon'][i+1]        
            # from wikipedia 
            # x1, y1 = self.transformer.llh2enu_2(lat, lon, 0, lat, lon, 0)
            # x2, y2 = self.transformer.llh2enu_2(lat2, lon2, 0, lat, lon, 0)
            # octopus llh2enu
            self.vehicle_gnss_trans.set_base(lat, lon)   
            enu_pt = self.vehicle_gnss_trans.latlon2xy(np.array([lat, lon]))
            enu_pt2 = self.vehicle_gnss_trans.latlon2xy(np.array([lat2, lon2]))
            enu = (np.array(enu_pt2) - np.array(enu_pt))
            x2, y2 = enu[0], enu[1]

            # print lat2, lon2
            # ll_pt = self.vehicle_gnss_trans.xy2latlon(np.array(enu_pt))
            # print ll_pt[0], ll_pt[1]
            # lat2_1, lon2_2 = xy2latlon(x2, y2)
            # print lat2_2, lon2_2
            self.data['yaw2'].append(np.rad2deg(np.arctan2(x2,y2)))
        self.data['yaw2'].append(self.data['yaw2'][-1])

        for k in self.data:
            self.data[k] = np.array(self.data[k])

    
