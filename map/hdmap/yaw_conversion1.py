from transformation_util import *

class YawConversion:
    def __init__(self):

        self.base_lat = 32.75707 
        self.base_lon = -111.55757
        self.base_gnss_trans = GNSSTransformer()
        self.base_gnss_trans.set_base(self.base_lat, self.base_lon)
        self.vehicle_gnss_trans = GNSSTransformer()
        self.base_North_GPS = self.base_gnss_trans.xy2latlon(np.array([[0., 0.], [0., 1.]]))
        
    # start from local north, x+1
    def inspvax_wrap(self, lat, lon, azimuth):
        x, y, z = self.base_gnss_trans.latlon2xy(np.array([self.base_lat, self.base_lon, 0]))
        self.vehicle_gnss_trans.set_base(lat, lon)
        delta, yaw = self.yaw_conversion(azimuth)
        return delta, yaw

    def yaw_conversion(self, azimuth):
        pts_enu = self.vehicle_gnss_trans.latlon2xy(self.base_North_GPS.copy())
        pts_enu = (pts_enu - pts_enu[0])[1]
        angle = np.arctan2(pts_enu[0], pts_enu[1])
        angle = np.rad2deg(angle)
        return angle, azimuth - angle


if __name__ == '__main__':
    s = YawConversion()
    print s.inspvax_wrap(32.78, -112.1, 300)