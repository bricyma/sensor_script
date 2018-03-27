from transformation_util import *

class YawConversion:
    def __init__(self):

        self.base_lat = 32.75707  # center between tucson and phoenix
        self.base_lon = -111.55757
        
        # self.base_lat = 32.754585 # sd
        # self.base_lon =  -117.175375
        self.base_gnss_trans = GNSSTransformer()
        self.base_gnss_trans.set_base(self.base_lat, self.base_lon)
        self.vehicle_gnss_trans = GNSSTransformer()
        self.base_North_GPS = self.base_gnss_trans.xy2latlon(np.array([[0., 0.], [1., 1.]]))
            

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

    # calculate the east, north position error between local world and global world (not accurate) 
    # check the ll2xy distortion
    # local (0, 0), (1, 1), compare with global
    def pos_conversion(self, lat, lon):
        self.vehicle_gnss_trans.set_base(lat, lon)   
        # xy => ll => xy
        pts_llh = self.vehicle_gnss_trans.xy2latlon(np.array([[0., 0.], [1., 1.]]))
        pts_enu = self.base_gnss_trans.latlon2xy(pts_llh)
        pts_enu = (pts_enu - pts_enu[0])[1]
        de, dn = 1 - pts_enu[0], 1 - pts_enu[1]
        dyaw = np.rad2deg(np.arctan2(pts_enu[0], pts_enu[1])) - 45
        if abs(de) < 0.0000001:
            print 'de = 0', lat, lon
        if abs(dn) < 0.0000001:
            print 'dn = 0', lat, lon
        return de, dn, dyaw

    # global (0, 0), (1, 1), compare with local
    def pos_conversion2(self, lat, lon):
        self.vehicle_gnss_trans.set_base(lat, lon)   
        # ll => xy
        pts_enu = self.vehicle_gnss_trans.latlon2xy(self.base_North_GPS)
        pts_enu = (pts_enu - pts_enu[0])[1]
        de, dn = 1 - pts_enu[0], 1 - pts_enu[1]
        dyaw = np.rad2deg(np.arctan2(pts_enu[0], pts_enu[1])) - 45
        return de, dn, dyaw

    # global x,y, x2, y2 => lat, lon2, lat2, lon2 => local x,y, x2, y2 => yaw
    def pos_conversion3(self, x, y, x2, y2):
        pts_llh = self.base_gnss_trans.xy2latlon(np.array([x, y]))
        self.vehicle_gnss_trans.set_base(pts_llh[0], pts_llh[1])
        
        pts_llh2 = self.base_gnss_trans.xy2latlon(np.array([x2, y2]))
        pts_enu2 = self.vehicle_gnss_trans.latlon2xy(pts_llh2)
        yaw = np.rad2deg(np.arctan2(pts_enu2[0], pts_enu2[1]))
        return yaw



if __name__ == '__main__':
    s = YawConversion()
    print s.inspvax_wrap(32.78, -112.1, 300)