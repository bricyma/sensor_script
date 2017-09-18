import rosbag
import numpy as np
from matplotlib import pyplot as plt
import math

EARTH_RADIUS = 6378137.0
class LocCheck:
    def __init__(self, bagname):
        self.bag = rosbag.Bag(bagname)
        self.data = {}
        self.baseLat = 0.693143165823
        self.baseLon = 2.04736661229
        self.gnss_en = []
        self.image_en = []
        self.diff_en = []
        self.diff_xy = []
        self.yaw = []

    def import_bag(self):
        self.data['gnss'] = {}
        self.data['image'] = {}
        self.data['gnss']['ll'] = {}  # latitude, longitude
        self.data['gnss']['xy'] = {}  # base_link
        self.data['gnss']['en'] = {}  # earth north frame
        self.data['image']['ll'] = {}
        self.data['image']['xy'] = {}
        self.data['image']['en'] = {}

        unit = {}
        for topic, msg, t in self.bag.read_messages(topics=['/image_localization_fusion/inspvax']):
            unit_ll = self.inspvax2dict(msg)
            unit_en = self.en2dict(msg, *self.latlon2xy(msg.latitude, msg.longitude))

            self.data['image']['ll'][str(unit_ll['seq'])] = unit_ll
            self.data['image']['en'][str(unit_en['seq'])] = unit_en

        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            unit_ll = self.inspvax2dict(msg)
            unit_en = self.en2dict(msg, *self.latlon2xy(msg.latitude, msg.longitude))

            self.data['gnss']['ll'][str(unit_ll['seq'])] = unit_ll
            self.data['gnss']['en'][str(unit_en['seq'])] = unit_en

        # dict to array
        list_gnss_en = []
        list_image_en = []
        list_yaw = []
        for item in sorted(self.data['gnss']['en'].iterkeys()):
            if item in self.data['image']['en']:
                list_gnss_en.append((self.data['gnss']['en'][item]['e'], self.data['gnss']['en'][item]['n']))
                list_image_en.append((self.data['image']['en'][item]['e'], self.data['image']['en'][item]['n']))
                list_yaw.append(self.data['gnss']['en'][item]['yaw'])
        self.gnss_en = np.array(list_gnss_en)
        self.image_en = np.array(list_image_en)
        self.diff_en = self.image_en - self.gnss_en
        self.yaw = self.DEG2RAD(np.array(list_yaw))

        # enu to base_link
        x = self.diff_en[:, 0]
        y = self.diff_en[:, 1]
        self.diff_xy = np.zeros([len(x), 2])
        self.diff_xy[:, 0] = -x * np.cos(self.yaw) + y * np.sin(self.yaw)
        self.diff_xy[:, 1] = x * np.sin(self.yaw) + y * np.cos(self.yaw)


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



    def plot(self):
        plt.subplot(211)
        plt.plot(self.diff_en[:, 0], label='e')
        plt.plot(self.diff_en[:, 1], label='n')
        plt.legend(loc='upper left')
        plt.subplot(212)
        plt.plot(self.diff_xy[:, 0], label='x')
        plt.plot(self.diff_xy[:, 1], label='y')
        plt.legend(loc='upper left')

        plt.show()


    @staticmethod
    def inspvax2dict(gps_msg):
        d = {
            'seq': gps_msg.header2.seq,
            'latitude': gps_msg.latitude,
            'longitude': gps_msg.longitude,
            'altitude': gps_msg.altitude,
            'yaw': gps_msg.azimuth,
            'north_velocity': gps_msg.north_velocity,
            'east_velocity': gps_msg.east_velocity,
            'system_ts': int(gps_msg.header2.stamp.secs * 1e9) + int(gps_msg.header2.stamp.nsecs),
            'gps_week': int(gps_msg.header.gps_week),
            'gps_week_seconds': int(gps_msg.header.gps_week_seconds)
        }
        return d

    @staticmethod
    def en2dict(gps_msg, x, y):
        d = {
            'seq': gps_msg.header2.seq,
            'e': x,
            'n': y,
            'u': gps_msg.altitude,
            'yaw': gps_msg.azimuth,
            'north_velocity': gps_msg.north_velocity,
            'east_velocity': gps_msg.east_velocity,
            'system_ts': int(gps_msg.header2.stamp.secs * 1e9) + int(gps_msg.header2.stamp.nsecs),
            'gps_week': int(gps_msg.header.gps_week),
            'gps_week_seconds': int(gps_msg.header.gps_week_seconds)
        }
        return d

    def run(self):
        self.import_bag()
        self.plot()


if __name__ == '__main__':
    bag = '2017-09-18-10-38-12.bag'
    prefix = '/home/zhibei/workspace/rosbag/'
    check = LocCheck(prefix+bag)
    check.run()