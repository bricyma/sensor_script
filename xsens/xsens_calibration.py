import sys
import os
from dataset_store import Dataset
import numpy as np
import matplotlib.pyplot as plt


class DataParser:
    def __init__(self, info):
        self.bag_name = info['bag_name']
        self.ts_begin = info['ts_begin']
        self.ts_end = info['ts_end']

    def parse(self):
        ds = Dataset.open(self.bag_name)
        xsens_data, novatel_data = [], []
        for (ts, ins), (ts, xsens) in ds.fetch_aligned('/novatel_data/inspvax',
                                                       '/xsens_driver/imupos',
                                                       ts_begin=self.ts_begin, ts_end=self.ts_end):
            xsens_data.append(xsens)
            novatel_data.append(ins)
        return novatel_data, xsens_data


class Analyzer:
    def __init__(self, info):
        parser = DataParser(info)
        file_name = info['bag_name']
        ts_begin = info['ts_begin']
        ts_end = info['ts_end']
        self.data = {'novatel': {'pitch': [], 'roll': [], 'yaw': []},
                     'xsens': {'pitch': [], 'roll': [], 'yaw': []}}
        self.delta = {'pitch': [], 'roll': [], 'yaw': []}
        novatel_data, xsens_data = parser.parse()
        self.data_warp(novatel_data, xsens_data)
        self.plot()

    def data_warp(self, novatel_data, xsens_data):
        index = []
        for frame in novatel_data:
            self.data['novatel']['pitch'].append(frame.pitch)
            self.data['novatel']['roll'].append(frame.roll)
            self.data['novatel']['yaw'].append(frame.azimuth)
        i = 0
        for frame in xsens_data:
            self.data['xsens']['pitch'].append(frame.pitch)
            self.data['xsens']['roll'].append(frame.roll)
            self.data['xsens']['yaw'].append(frame.yaw)
            if np.sqrt(frame.velX ** 2 + frame.velY ** 2) > 10:
                index.append(i)
            i += 1
        print len(self.data['xsens']['pitch'])
        print index
        for sensor in self.data:
            for item in self.data[sensor]:
                self.data[sensor][item] = np.array(self.data[sensor][item])
                # self.data[sensor][item] = self.data[sensor][item][index]

        for item in self.data['novatel']:
            self.delta[item] = self.data['novatel'][item] - self.data['xsens'][item]
            print 'mean of ', item, np.mean(self.delta[item])

    def plot(self):
        plt.subplot(311)
        plt.plot(self.delta['pitch'], 'r', label='pitch')
        plt.legend(loc='upper left')
        plt.subplot(312)
        plt.plot(self.delta['roll'], 'r', label='roll')
        plt.legend(loc='upper left')
        plt.subplot(313)
        plt.plot(self.delta['yaw'], 'r', label='yaw')
        plt.legend(loc='upper left')
        plt.show()


if __name__ == '__main__':
    # bag = "2018-06-06-18-35-04"
    bag = "2018-05-31-17-50-10"
    bag = "2018-05-21-11-17-12"
    bag = "2018-05-30-13-04-45"
    bag = "2018-05-29-14-41-16"
    ts_begin = "1:00"
    ts_end = "100:00"
    info = {'bag_name': bag, 'ts_begin': ts_begin, 'ts_end': ts_end}
    a = Analyzer(info)
