import rosbag
import math
import matplotlib.pyplot as plt
import sys
import numpy as np


def DEG2RAD(self, x):
    return x / (180 / math.pi)


def RAD2DEG(self, x):
    return x * (180 / math.pi)

    # method 1


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
