
# #!/usr/bin/env python

# analyze the relationship between postion deviation and yaw offset
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
from llh2enu.llh2enu_gps_transformer import *
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from transformation_util import GNSSTransformer
from gnss_transformer import GPSTransformer


class IMUAnalyzer:
    def __init__(self, info, flag):
        parser = DataParser(info)
        self.file_name = info['bag_name']
        self.ts_begin = info['ts_begin']
        self.ts_end = info['ts_end']

        self.base_lat = 32.75707  # center between tucson and phoenix
        self.base_lon = -111.55757

        self.octopus_gnss_trans = GNSSTransformer()
        self.wiki_gnss_trans = GPSTransformer()
        self.octopus_gnss_trans.set_base(self.base_lat, self.base_lon)
        self.wiki_gnss_trans.set_base(self.base_lat, self.base_lon)
        # get bestpos, insspd, inspvax from dataset
        # get ros message data
        corrimu_data1, bestpos_data1, spd_data1, ins_data1 = parser.parse_all()
        imu1 = {'corrimu': corrimu_data1, 'bestpos': bestpos_data1,
                'ins': ins_data1, 'spd': spd_data1}

        self.data1 = self.data_warp(imu1)
        self.analysis()

    def analysis(self):
        items = ['x_acc', 'y_acc', 'z_acc',
                 'pitch_rate', 'roll_rate', 'yaw_rate']
        for item in items:
            print 'std of ' + item, format(np.std(self.data1['corrimu'][item]), '.3f')
        print 'mean of x offset: ', format(np.mean(self.data1['offset']['x']), '.4f')
        print 'mean of y offset: ', format(np.mean(self.data1['offset']['y']), '.3f')
        print 'std of pitch', format(np.std(self.data1['ins']['pitch']), '.3f')
        print 'std of roll', format(np.std(self.data1['ins']['roll']), '.3f')
        print 'mean of pos x offset: ', format(np.mean(self.data1['pos']['x']), '.4f')
        print 'mean of pos y offset: ', format(np.mean(self.data1['pos']['y']), '.3f')

    def data_warp(self, imu):
        pos = {'x': [], 'y': []}
        offset = {'x': [], 'y': []}
        corrimu = {'t': [], 'x_acc': [], 'y_acc': [], 'z_acc': [],
                   'pitch_rate': [], 'roll_rate': [], 'yaw_rate': []}
        bestpos = {'x': [], 'y': [], 't': [],
                   'diff_age': [], 'status': [], 'sol_age': []}
        ins = {'lat': [], 'lon': [], 'pitch': [], 'roll': [], 'x': [], 'y': [], 'yaw': [],
               't': [], 't_gps': [], 'lat_std': [], 'lon_std': [], 'status': []}
        spd = {'yaw': [], 't': [], 't_gps': []}
        hdmap = {'x': [], 'y': [], 'yaw': []}
        data = {'corrimu': corrimu, 'bestpos': bestpos,
                'ins': ins, 'spd': spd, 'offset': offset, 'pos': pos, 'map': hdmap}
        # initilize base point, the middle point between start and end point
        transformer = gps_transformer()
        # self.base_lat, self.base_lon = 0.5 * (imu['ins'][0].latitude + imu['ins'][-1].latitude), 0.5 * (
        # imu['ins'][0].longitude + imu['ins'][-1].longitude)
        data_rate = 125  # IMU-IGM-S1
        for msg in imu['corrimu']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            data['corrimu']['t'].append(pc_time)
            data['corrimu']['x_acc'].append(msg.x_accel * data_rate)
            data['corrimu']['y_acc'].append(msg.y_accel * data_rate)
            data['corrimu']['z_acc'].append(msg.z_accel * data_rate)
            data['corrimu']['pitch_rate'].append(msg.pitch_rate * data_rate)
            data['corrimu']['roll_rate'].append(msg.roll_rate * data_rate)
            data['corrimu']['yaw_rate'].append(msg.yaw_rate * data_rate)

        for msg in imu['spd']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            novatel_time = 315964800 + msg.header.gps_week * 7 * 24 * 60 * 60 \
                + float(msg.header.gps_week_seconds) / \
                1000 - 18  # GPS time to epoch second
            data['spd']['yaw'].append(msg.track_ground)
            data['spd']['t'].append(pc_time)
            data['spd']['t_gps'].append(novatel_time)
        for msg in imu['bestpos']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            x, y = transformer.llh2enu_5(
                msg.latitude, msg.longitude, 0, self.base_lat, self.base_lon, 0)
            data['bestpos']['x'].append(x)
            data['bestpos']['y'].append(y)
            data['bestpos']['t'].append(pc_time)
            data['bestpos']['diff_age'].append(msg.diff_age)
            data['bestpos']['sol_age'].append(msg.sol_age)
            data['bestpos']['status'].append(msg.position_type)
        for msg in imu['ins']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            novatel_time = 315964800 + msg.header.gps_week * 7 * 24 * 60 * 60 \
                + float(msg.header.gps_week_seconds) / \
                1000 - 18  # GPS time to epoch second
            x, y = transformer.llh2enu_5(
                msg.latitude, msg.longitude, 0, self.base_lat, self.base_lon, 0)
            data['ins']['x'].append(x)
            data['ins']['y'].append(y)
            data['ins']['lat'].append(msg.latitude)
            data['ins']['lon'].append(msg.longitude)
            data['ins']['yaw'].append(msg.azimuth)
            data['ins']['roll'].append(msg.roll)
            data['ins']['pitch'].append(msg.pitch)
            data['ins']['t'].append(pc_time)
            data['ins']['t_gps'].append(novatel_time)
            data['ins']['lat_std'].append(msg.latitude_std)
            data['ins']['lon_std'].append(msg.longitude_std)
            data['ins']['status'].append(msg.position_type)
        # list to numpy array
        for topic in data:
            for item in data[topic]:
                if topic is not 'map':
                    data[topic][item] = np.array(data[topic][item])

        # find data between (32.146004, -110.893713) to (32.246111, -110.990079)
        # Tucson => Phoneix
        start_lat, start_lon = 32.146004, -110.893713
        end_lat, end_lon = 32.246111, -110.990079
        selected_id = []
        for i in range(len(data['ins']['lat'])):
            if start_lat < data['ins']['lat'][i] < end_lat and end_lon < data['ins']['lon'][i] < start_lon \
                    and (270 < data['ins']['yaw'][i] < 360 or 0 < data['ins']['yaw'][i] < 10):
                selected_id.append(i)
        selected_id = np.array(selected_id)
        for topic in data:
            for item in data[topic]:
                if len(data[topic][item]) > 0:
                    data[topic][item] = data[topic][item][selected_id]

        x_offset, y_offset = self.calculate_offset(data)
        # leverarm check
        data['offset']['x'], data['offset']['y'] = x_offset, y_offset
        # position offset check
        data, data['pos']['x'], data['pos']['y'] = self.check_pos(data)
        return data

    # leverarm check
    def calculate_offset(self, data):
        x1, y1 = data['bestpos']['x'], data['bestpos']['y']
        x2, y2 = data['ins']['x'], data['ins']['y']

        x_offset, y_offset = self.get_distance(
            x1, y1, x2, y2, data['ins']['yaw'])

        x_offset[x_offset > 2] = 0
        y_mean = np.mean(y_offset)
        return x_offset, y_offset

    # distance between INSPVAX and HDmap lane center
    def check_pos(self, data):
        map_file = "I-10_Tucson2Phoenix_20180412.hdmap"
        # map_file = "I-10_Phoenix2Tucson_20180412.hdmap"
        with open(map_file, 'rb') as f:
            submap = f.read()
        hdmap = TSMap(submap)
        x1, y1 = data['bestpos']['x'], data['bestpos']['y']
        x2, y2 = [], []
        xy = []
        for i in range(len(x1)):
            x, y = x1[i], y1[i]
            p = Point3d(x, y)
            try:
                ref_p = hdmap.get_ref_pt(p)
            except:
                ref_p = p
            x2.append(ref_p.x)
            y2.append(ref_p.y)
            xy.append([ref_p.x, ref_p.y])

        # get the distance between lane center and bestpos
        x2, y2 = np.array(x2), np.array(y2)
        x_offset, y_offset = self.get_distance(
            x1, y1, x2, y2, data['ins']['yaw'])

        # yaw angle of map's lane        
        xy = np.array(xy)
        ll = self.octopus_gnss_trans.xy2latlon(xy)
        # wiki_xy = self.wiki_gnss_trans.latlon2xy(ll)

        for i in range(len(ll)-1):
            lat, lon = ll[i][0], ll[i][1]
            lat2, lon2 = ll[i+1][0], ll[i+1][1]
            self.wiki_gnss_trans.set_base(lat, lon)
            enu_pt_0 = self.wiki_gnss_trans.latlon2xy(np.array([lat, lon]))
            enu_pt = self.wiki_gnss_trans.latlon2xy(np.array([lat2, lon2]))
            enu_pt = enu_pt - enu_pt_0
            delta_x, delta_y = enu_pt[0], enu_pt[1]
            data['map']['yaw'].append(np.rad2deg(np.arctan2(delta_x, delta_y)))

        # one base, llh2enu
        # for xy in wiki_xy:
        #     data['map']['x'].append(xy[0])
        #     data['map']['y'].append(xy[1])
        # data['map']['x'] = np.array(data['map']['x'])
        # data['map']['y'] = np.array(data['map']['y'])
        # data['map']['yaw'] = np.rad2deg(np.arctan2(
        #     data['map']['x'][1:] - data['map']['x'][:-1], data['map']['y'][1:] - data['map']['y'][:-1])) 

        for i in range(len(data['map']['yaw'])-1):
            if abs(data['map']['yaw'][i]) == 0:
                data['map']['yaw'][i] = min(data['map']['yaw'][i-1], data['map']['yaw'][i+1])
        

        data['map']['yaw'] = np.append(
            data['map']['yaw'], data['map']['yaw'][-1])
        data['map']['yaw'] += 360
        return data, x_offset, y_offset

    # input: x1, y1, x2, y2, yaw
    # output: x offset, y offset
    def get_distance(self, x1, y1, x2, y2, yaw):
        abs_offset = np.sqrt((x1 - x2)**2 + (y1 - y2)**2)
        k = np.tan(np.deg2rad(90 - yaw))
        A = k
        B = -1
        C1 = -k * x1 + y1
        C2 = -k * x2 + y2
        # calculate the x offset
        x_offset = abs(C1 - C2) / np.sqrt(A ** 2 + B ** 2)
        y_offset = np.sqrt(abs_offset**2 - x_offset ** 2)
        return x_offset, y_offset


if __name__ == '__main__':
    bag_name = '2018-05-16-15-43-19'

    ts_begin = '0:59'
    ts_end = '100:01'
    info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
    analyzer = IMUAnalyzer(info, 1)
