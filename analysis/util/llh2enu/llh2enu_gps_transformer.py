#!/usr/bin/env python
# created by Fuheng Deng on 7/5/2017
# temporarily dealing with only one llh input

import numpy as np
import math

# define constants
a = 6378137.0
b = 6356752.3142
EARTH_RADIUS = 6378137.0
e2 = 1 - (b/a) * (b/a)
f = (a-b)/a
e_sq = f * (2 - f)

class gps_transformer():

    def __init__(self):
        self.init_phi = 10000
        self.init_lam = 10000
        self.init_h = -1.0

    @staticmethod
    def deg2rad(deg):
        return deg * math.pi / 180

    #method referenced http://digext6.defence.gov.au/dspace/bitstream/1947/3538/1/DSTO-TN-0432.pdf
    def llh2enu_1(self, lat, lon, h, lat0, lon0, h0):
        # get current llh difference
        phi = self.deg2rad(lat)
        lam = self.deg2rad(lon)
        phi0 = self.deg2rad(lat0)
        lam0 = self.deg2rad(lon0)
        dphi = phi - phi0
        dlam = lam - lam0
        dh = h - h0
        # useful representation
        cl = math.cos(lam0)
        sl = math.sin(lam0)
        cp = math.cos(phi0)
        sp = math.sin(phi0)
        tmp = math.sqrt(1-e2*sp*sp)
        # tranformation
        de = (a / tmp + h) * cp * dlam - (a * (1 - e2) / (tmp * tmp * tmp) + h) * sp * dphi * dlam + cp * dlam * dh
        dn = (a * (1 - e2) / (tmp * tmp * tmp) + h) * dphi  + 1.5 * cp * sp * a * e2 * dphi * dphi  + sp * sp * dh * dphi + 0.5 * sp * cp * (a / tmp  + h) * dlam * dlam
        #du = dh - 0.5 * (a - 1.5 * a * e2 * cp * cp  + 0.5 * a * e2 + h) * dphi * dphi  - 0.5 * cp * cp * (a / tmp - h) * dlam * dlam
        denu = (de, dn)
        return denu

    # method referecenced wiki_pedia
    def llh2ecef(self, lat, lon, h):
        phi, lam = self.deg2rad(lat), self.deg2rad(lon)
        s = math.sin(phi)
        N = a / math.sqrt(1 - e_sq * s * s)
        sin_lam = math.sin(lam)
        cos_lam = math.cos(lam)
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        return (h + N) * cos_phi * cos_lam, \
               (h + N) * cos_phi * sin_lam, \
               (h + (b * b) / (a * a) * N) * sin_phi

    def ecef2enu(self, x, y, z, lat0, lon0, h0):
        phi, lam = self.deg2rad(lat0), self.deg2rad(lon0)
        s = math.sin(phi)
        N = a / math.sqrt(1 - e_sq * s * s)
        sin_lam = math.sin(lam)
        cos_lam = math.cos(lam)
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        x0 = (h0 + N) * cos_phi * cos_lam
        y0 = (h0 + N) * cos_phi * sin_lam
        z0 = (h0 + (1 - e_sq) * N) * sin_phi
        xd = x - x0
        yd = y - y0
        zd = z - z0
        return -sin_lam * xd + cos_lam * yd, \
               -cos_lam * sin_phi * xd - sin_phi * sin_lam * yd + cos_phi * zd, \
               cos_phi * cos_lam * xd + cos_phi * sin_lam * yd + sin_phi * zd



    def DEG2RAD(self, x):
        return x / (180 / math.pi)

    def RAD2DEG(self, x):
        return x * (180 / math.pi)

    # octopus transformation
    def llh2enu5(self, lat_, lon_, baseLat, baseLon):
        lat, lon = self.DEG2RAD(lat_), self.DEG2RAD(lon_)
        baseLat = self.DEG2RAD(baseLat)
        baseLon = self.DEG2RAD(baseLon)
        xx = math.cos(lat) * math.cos(lon) * math.cos(baseLon) * math.cos(baseLat) \
            + math.cos(lat) * math.sin(lon) * math.sin(baseLon) * math.cos(baseLat) \
            + math.sin(lat) * math.sin(baseLat)
        yy = -math.cos(lat) * math.cos(lon) * math.sin(baseLon) \
            + math.cos(lat) * math.sin(lon) * math.cos(baseLon)
        zz = -math.cos(lat) * math.cos(lon) * math.cos(baseLon) * math.sin(baseLat) \
             - math.cos(lat) * math.sin(lon) * math.sin(baseLon) * math.sin(baseLat) \
            + math.sin(lat) * math.cos(baseLat)
        x = math.atan2(yy, xx) * EARTH_RADIUS
        y = math.log(math.tan(math.asin(zz) / 2 + math.pi / 4)) * EARTH_RADIUS
        return x, y


    def llh2enu_2(self, lat, lon, h, lat0, lon0, h0):
        x_, y_, z_ = self.llh2ecef(lat, lon, h)
        x, y, _ = self.ecef2enu(x_, y_, z_, lat0, lon0, h0)
        return x, y

    def llh2enu_3(self, lat_, lon_, baseLat_, baseLon_):
        s = math.cos(baseLat_ * math.pi / 180)
        x = s * a * math.pi * lon_ / 180
        y = s * a * math.log(math.tan(math.pi * (90 + lat_) / 360))
        return x, y

    #def compute_distance(self, ):

if __name__ == '__main__':
    test = gps_transformer()
    lat = 30
    lon = 100
    lat0 = 29
    lon0 = 99

    x5, y5 = test.llh2enu5(lat, lon, lat0, lon0)
    x5_0, y5_0 = test.llh2enu5(lat0, lon0, lat0, lon0)
    d_x5, d_y5 = x5 - x5_0, y5-y5_0

    x2, y2 = test.llh2enu_2(lat, lon, 0, lat0, lon0, 0)
    x2_0, y2_0 = test.llh2enu_2(lat0, lon0, 0, lat0, lon0, 0)
    d_x2, d_y2 = x2-x2_0, y2-y2_0

    x1, y1 = test.llh2enu_1(lat, lon, 0, lat0, lon0, 0)
    x1_0, y1_0 = test.llh2enu_1(lat0, lon0, 0, lat0, lon0, 0)
    d_x1, d_y1 = x1-x1_0, y1-y1_0

    x3, y3 = test.llh2enu_3(lat, lon, lat0, lon0)
    x3_0, y3_0 = test.llh2enu_3(lat0, lon0, lat0, lon0)
    d_x3, d_y3 = x3 - x3_0, y3-y3_0

    x4, y4 = test.llh2enu_4(lat, lon, 0, lat0, lon0, 0)
    x4_0, y4_0 = test.llh2enu_4(lat0, lon0, 0, lat0, lon0, 0)
    d_x4, d_y4 = x4-x4_0, y4-y4_0

    print d_x5, d_y5
    print d_x2, d_y2
    print d_x1, d_y1
    print d_x3, d_y3
    print d_x4, d_y4
