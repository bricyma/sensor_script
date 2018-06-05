# #!/usr/bin/env python

# analyze the relationship between postion deviation and yaw offset
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
from tsmap import TSMap, Lane, Point3d, Bound, latlon2xy
from transformation_util import GNSSTransformer
from gnss_transformer import GPSTransformer
import json
import copy


class IMUAnalyzer:
    def __init__(self, info):
        # test bag's info
        parser = DataParser(info)
        self.file_name = info['bag_name']
        self.ts_begin = info['ts_begin']
        self.ts_end = info['ts_end']

        # LLH to ENU base point, also the base point of Hdmap
        self.base_lat = 32.75707  # center between tucson and phoenix
        self.base_lon = -111.55757

        self.octopus_gnss_trans = GNSSTransformer()
        self.wiki_gnss_trans = GPSTransformer()
        self.octopus_gnss_trans.set_base(self.base_lat, self.base_lon)
        self.wiki_gnss_trans.set_base(self.base_lat, self.base_lon)

        data_matrix = {'acc_x': [], 'acc_y': [], 'acc_z': [], 'pitch_rate': [],
                       'roll_rate': [], 'yaw_rate': [], 'yaw_rate_change': [], 't_ros': [], 't_gps': []}
        self.matrix_keys = ['acc_x', 'acc_y', 'acc_z', 'pitch_rate',
                            'roll_rate', 'yaw_rate', 'yaw_rate_change', 't_ros', 't_gps']
        self.matrix = {'std': data_matrix, 'mean': copy.deepcopy(data_matrix)}

        # get corrimudata, bestpos, insspd, inspvax from dataset
        # get ros message data
        corrimu_data1, bestpos_data1, spd_data1, ins_data1 = parser.parse_all()
        imu1 = {'corrimu': corrimu_data1, 'bestpos': bestpos_data1,
                'ins': ins_data1, 'spd': spd_data1}

        # warp data
        self.data1 = self.data_warp(imu1)
        self.analysis()

    # print analytical result
    def analysis(self):
        print '\n'
        print self.file_name
        print '******BENCHMARK*********'
        items = ['acc_x', 'acc_y', 'acc_z',
                 'pitch_rate', 'roll_rate', 'yaw_rate']
        orientation = ['pitch', 'roll', 'yaw']
        for item in items:
            self.matrix['std'][item] = format(
                np.std(self.data1['corrimu'][item]), '.6f')
            self.matrix['mean'][item] = format(
                np.mean(self.data1['corrimu'][item]), '.6f')
        for item in orientation:
            self.matrix['std'][item] = format(
                np.std(self.data1['ins'][item]), '.6f')
            self.matrix['mean'][item] = format(
                np.mean(self.data1['ins'][item]), '.6f')

        # delta yaw rate
        yaw_rate_chagne = self.data1['corrimu']['yaw_rate'][1:] - \
            self.data1['corrimu']['yaw_rate'][:-1]
        self.matrix['std']['yaw_rate_change'] = format(
            np.std(yaw_rate_chagne), '.6f')
        self.matrix['mean']['yaw_rate_change'] = format(
            np.mean(yaw_rate_chagne), '.6f')

        # ros timestamp and gps timestamp
        delta_t_ros = self.data1['ins']['t_ros'][1:] - \
            self.data1['ins']['t_ros'][:-1]
        delta_t_gps = self.data1['ins']['t_gps'][1:] - \
            self.data1['ins']['t_gps'][:-1]
        self.matrix['std']['t_ros'] = format(np.std(delta_t_ros), '.6f')
        self.matrix['std']['t_gps'] = format(np.std(delta_t_gps), '.6f')
        self.matrix['mean']['t_ros'] = format(np.mean(delta_t_ros), '.6f')
        self.matrix['mean']['t_gps'] = format(np.mean(delta_t_gps), '.6f')

        for category in self.matrix:
            print category
            for item in self.matrix_keys:
                # print self.matrix[category][item]
                pass
        # print '*******LEVERARM CALIBRATION********'
        # print 'mean of x offset: ', format(np.mean(self.data1['offset']['x']), '.4f')
        # print 'mean of y offset: ', format(np.mean(self.data1['offset']['y']), '.4f')
        # print '*******POSITION OFFSET*********'
        # print 'mean of pos x offset: ', format(np.mean(self.data1['pos']['x']), '.4f')
        # print 'mean of pos y offset: ', format(np.mean(self.data1['pos']['y']), '.4f')
        # print '*******CALIBRATION VEHICLEBODYROTATION**********'
        diff = self.data1['ins']['yaw'] - self.data1['spd']['yaw']
        diff[abs(diff) > 1] = 0
        # print 'mean of (ins yaw - spd yaw): ', format(np.mean(diff), '.6f')
        diff_ins = self.data1['ins']['yaw'] - self.data1['map']['yaw']
        diff_spd = self.data1['spd']['yaw'] - self.data1['map']['yaw']
        # filter
        diff_ins[abs(diff_ins) > 1] = 0
        diff_spd[abs(diff_spd) > 1] = 0
        # print 'average of (ins -map): ', format(np.mean(diff_ins), '.4f')
        # print 'average of (spd -map): ', format(np.mean(diff_spd), '.4f')

    # TODO
    # check parameter satisfies standard
    def check_data():
        with open('conf.json') as f:
            data = json.load(f)

    def data_warp(self, imu):
        pos = {'x': [], 'y': []}
        offset = {'x': [], 'y': []}
        corrimu = {'t_ros': [], 'acc_x': [], 'acc_y': [], 'acc_z': [],
                   'pitch_rate': [], 'roll_rate': [], 'yaw_rate': []}
        bestpos = {'x': [], 'y': [], 't_ros': [],
                   'diff_age': [], 'status': [], 'sol_age': []}
        ins = {'lat': [], 'lon': [], 'pitch': [], 'roll': [], 'x': [], 'y': [], 'yaw': [],
               't_ros': [], 't_gps': [], 'lat_std': [], 'lon_std': [], 'status': []}
        spd = {'yaw': [], 't_ros': [], 't_gps': []}
        hdmap = {'x': [], 'y': [], 'yaw': []}
        data = {'corrimu': corrimu, 'bestpos': bestpos,
                'ins': ins, 'spd': spd, 'offset': offset, 'pos': pos, 'map': hdmap}

        # self.data_list = {'corrimu': [], 'bestpos':[], 'ins':[], 'spd':[], 'offset':[], 'pos':[], 'map':[]}
        data_rate = 125  # IMU-IGM-S1
        for msg in imu['corrimu']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            data['corrimu']['t_ros'].append(pc_time)
            data['corrimu']['acc_x'].append(msg.x_accel * data_rate)
            data['corrimu']['acc_y'].append(msg.y_accel * data_rate)
            data['corrimu']['acc_z'].append(msg.z_accel * data_rate)
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
            data['spd']['t_ros'].append(pc_time)
            data['spd']['t_gps'].append(novatel_time)
        for msg in imu['bestpos']:
            pc_time = float(str(msg.header2.stamp.secs)) \
                + float(str(msg.header2.stamp.nsecs)) * \
                1e-9  # epoch second
            enu_pt = self.octopus_gnss_trans.latlon2xy(
                np.array([msg.latitude, msg.longitude]))
            x, y = enu_pt[0], enu_pt[1]
            data['bestpos']['x'].append(x)
            data['bestpos']['y'].append(y)
            data['bestpos']['t_ros'].append(pc_time)
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
            enu_pt = self.octopus_gnss_trans.latlon2xy(
                np.array([msg.latitude, msg.longitude]))
            x, y = enu_pt[0], enu_pt[1]
            data['ins']['x'].append(x)
            data['ins']['y'].append(y)
            data['ins']['lat'].append(msg.latitude)
            data['ins']['lon'].append(msg.longitude)
            data['ins']['yaw'].append(msg.azimuth)
            data['ins']['roll'].append(msg.roll)
            data['ins']['pitch'].append(msg.pitch)
            data['ins']['t_ros'].append(pc_time)
            data['ins']['t_gps'].append(novatel_time)
            data['ins']['lat_std'].append(msg.latitude_std)
            data['ins']['lon_std'].append(msg.longitude_std)
            data['ins']['status'].append(msg.position_type)
        # list to numpy array
        for topic in data:
            for item in data[topic]:
                if topic is not 'map':
                    data[topic][item] = np.array(data[topic][item])

        # Obtain data in the chosen route
        # find data between start: (32.146004, -110.893713) to end: (32.246111, -110.990079)
        # Tucson => Phoneix
        start_lat, start_lon = 32.146004, -110.893713
        end_lat, end_lon = 32.246111, -110.990079
        selected_id = []
        for i in range(len(data['ins']['lat'])):
            if start_lat < data['ins']['lat'][i] < end_lat and end_lon < data['ins']['lon'][i] < start_lon \
                    and (270 < data['ins']['yaw'][i] < 360 or 0 < data['ins']['yaw'][i] < 10):
                selected_id.append(i)
        selected_id = np.array(selected_id)
        if len(selected_id) == 0:
            return
        else:
            print 'length of benchmark: ', len(selected_id)
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

        for i in range(len(ll) - 1):
            lat, lon = ll[i][0], ll[i][1]
            lat2, lon2 = ll[i + 1][0], ll[i + 1][1]
            self.wiki_gnss_trans.set_base(lat, lon)
            enu_pt_0 = self.wiki_gnss_trans.latlon2xy(np.array([lat, lon]))
            enu_pt = self.wiki_gnss_trans.latlon2xy(np.array([lat2, lon2]))
            enu_pt = enu_pt - enu_pt_0
            delta_x, delta_y = enu_pt[0], enu_pt[1]
            data['map']['yaw'].append(np.rad2deg(np.arctan2(delta_x, delta_y)))

        for i in range(len(data['map']['yaw']) - 1):
            if abs(data['map']['yaw'][i]) == 0:
                data['map']['yaw'][i] = min(
                    data['map']['yaw'][i - 1], data['map']['yaw'][i + 1])
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
