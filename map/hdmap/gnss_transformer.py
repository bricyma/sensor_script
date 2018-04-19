import math
import numpy as np
from llh2enu.llh2enu_gps_transformer import *
from transformation_util import GNSSTransformer, get_rotation_translation_mat

# define constants
a = 6378137.0
b = 6356752.3142
e2 = 1 - (b / a) * (b / a)
f = (a - b) / a
e_sq = f * (2 - f)


class GPSTransformer:
    def __init__(self):
        self.er = 6378137.0
        # set default base
        self.base_lat, self.base_lon = None, None
        self.set_base(0, 0)

    def set_base(self, base_lat, base_lon):
        self.base_lat, self.base_lon = base_lat, base_lon

    def latlon2xy(self, pts):
        is_vector_input = len(pts.shape) < 2
        if is_vector_input:
            lat = pts[0]
            lon = pts[1]
            pts[0], pts[1] = self.llh2enu(lat, lon)
            return pts
        else:
            lat = pts[:, 0]
            lon = pts[:, 1]

        for i in range(0, len(lat)):
            x, y = self.llh2enu(lat[i], lon[i])
            pts[i][0], pts[i][1] = x, y
        return pts

    def llh2enu(self, lat, lon):
        h = 0
        x_, y_, z_ = self.llh2ecef(lat, lon, h)
        x, y, _ = self.ecef2enu(x_, y_, z_)
        return x, y

    # method referecenced wiki_pedia
    def llh2ecef(self, lat, lon, h):
        phi, lam = np.deg2rad(lat), np.deg2rad(lon)
        s = math.sin(phi)
        N = a / math.sqrt(1 - e_sq * s * s)
        sin_lam = math.sin(lam)
        cos_lam = math.cos(lam)
        cos_phi = math.cos(phi)
        sin_phi = math.sin(phi)
        return (h + N) * cos_phi * cos_lam, \
               (h + N) * cos_phi * sin_lam, \
               (h + (b * b) / (a * a) * N) * sin_phi

    def ecef2enu(self, x, y, z):
        lat0 = self.base_lat
        lon0 = self.base_lon
        h0 = 0
        phi, lam = np.deg2rad(lat0), np.deg2rad(lon0)
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


if __name__ == '__main__':
    base_lat = 32.75707123656  # center between tucson and phoenix
    base_lon = -111.55757123656
    cc = GPSTransformer()

    lat = 32.4002113023
    lon = -111.130179869
    lat2 = lat - 0.00002
    lon2 = lon - 0.00002
    # lat2 = 32.4002047143
    # lon2 = -111.130170684
    cc.set_base(lat, lon)

    enu_pt = cc.latlon2xy(np.array([lat, lon]))
    enu_pt2 = cc.latlon2xy(np.array([lat2, lon2]))
    enu = (np.array(enu_pt2) - np.array(enu_pt))
    print np.rad2deg(np.arctan2(enu[0], enu[1]))

    cc = GNSSTransformer()
    cc.set_base(lat, lon)
    enu_pt = cc.latlon2xy(np.array([lat, lon]))
    enu_pt2 = cc.latlon2xy(np.array([lat2, lon2]))
    enu = (np.array(enu_pt2) - np.array(enu_pt))
    print np.rad2deg(np.arctan2(enu[0], enu[1]))

    dd = gps_transformer()
    x, y = dd.llh2enu_2(lat, lon, 0, lat, lon, 0)
    x2, y2 = dd.llh2enu_2(lat2, lon2, 0, lat, lon, 0)
    print np.rad2deg(np.arctan2(x2 - x, y2 - y))
