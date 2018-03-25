from transformation_util import *


class YawConversion:
    def __init__(self):

        base_lat = 22
        base_lon = 43
        self.base_gnss_trans = GNSSTransformer()
        self.base_gnss_trans.set_base(base_lat, base_lon)
        self.base_North_GPS = self.base_gnss_trans.xy2latlon(np.array([[0., 0.], [0., 1.]]))
        print self.base_North_GPS
        self.vehicle_gnss_trans = GNSSTransformer()


    def inspvax_wrap(self, lat, lon, azimuth):
        x, y, z = self.base_gnss_trans.latlon2xy(np.array([lat, lon, 0]))
        self.vehicle_gnss_trans.set_base(lat, lon)
        yaw = self.yaw_conversion(azimuth)
        return yaw

    def yaw_conversion(self, azimuth):
        pts_enu = self.vehicle_gnss_trans.latlon2xy(self.base_North_GPS.copy())
        pts_enu = (pts_enu - pts_enu[0])[1]
        

        print pts_enu
        angle = np.arctan2(pts_enu[0], pts_enu[1])
        return -(np.deg2rad(azimuth) - angle)



if __name__ == '__main__':
    s = YawConversion()
    print s.inspvax_wrap(23, 45, 30)