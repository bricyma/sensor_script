import sys
import os
from dataset_store import Dataset

# input: bag_name, ts_begin, ts_end
# output: error data, localization pose, inspvax and bestvel


class DataParser:
    def __init__(self, info):
        self.bag_name = info['bag_name']
        self.ts_begin = info['ts_begin']
        self.ts_end = info['ts_end']

    def parse(self):
        ds = Dataset.open(self.bag_name)
        error_data = []
        gps_data = []
        bestvel_data = []
        pose_data = []
        for (ts, error), (ts, pose), (ts, gps), (ts, bestvel) in ds.fetch_aligned('/error_monitor/error_info',
                                                                                  '/localization/pose', '/novatel_data/inspvax', '/novatel_data/bestvel',
                                                                                  ts_begin=self.ts_begin, ts_end=self.ts_end):
            error_data.append(error)
            gps_data.append(gps)
            bestvel_data.append(bestvel)
            pose_data.append(pose)

        return error_data, pose_data, gps_data, bestvel_data

    def parse_insspd(self):
        ds = Dataset.open(self.bag_name)
        insspd_data, inspvax_data, bestvel_data, pose_data, corrimu_data = [], [], [], [], []
        for (ts, insspd), (ts, inspvax), (ts, bestvel), (ts, pose), (ts, corrimu) in ds.fetch_aligned('/novatel_data/insspd',
                                                                                                      '/novatel_data/inspvax', '/novatel_data/bestvel', '/localization/pose',
                                                                                                      '/novatel_data/corrimudata',
                                                                                                      ts_begin=self.ts_begin, ts_end=self.ts_end):
            insspd_data.append(insspd)
            inspvax_data.append(inspvax)
            bestvel_data.append(bestvel)
            pose_data.append(pose)
            corrimu_data.append(corrimu)
        return insspd_data, inspvax_data, bestvel_data, pose_data, corrimu_data

    def parse_rtkpos(self):
        ds = Dataset.open(self.bag_name)
        rtkpos_data, inspvax_data = [], []
        for (ts, rtkpos), (ts, inspvax) in ds.fetch_aligned('/novatel_data/bestpos', '/novatel_data/inspvax',
                                                            ts_begin=self.ts_begin, ts_end=self.ts_end):
            rtkpos_data.append(rtkpos)
            inspvax_data.append(inspvax)
        return rtkpos_data, inspvax_data

    def parse_all(self):
        ds = Dataset.open(self.bag_name)
        corrimu_data, bestpos_data, spd_data, ins_data = [], [], [], []
        for (ts, corrimu), (ts, bestpos), (ts, insspd), (ts, inspvax) in ds.fetch_aligned('/novatel_data/corrimudata',
                                                                                          '/novatel_data/bestpos', '/novatel_data/insspd', '/novatel_data/inspvax',
                                                                                          ts_begin=self.ts_begin, ts_end=self.ts_end):
            corrimu_data.append(corrimu)
            bestpos_data.append(bestpos)
            spd_data.append(insspd)
            ins_data.append(inspvax)
        return corrimu_data, bestpos_data, spd_data, ins_data

    # parse dual novatel's info
    def parse_dual(self):
        ds = Dataset.open(self.bag_name)
        # ds = Dataset('/mnt/truenas/scratch/zhibei/b2_data_imu/2018-05-11-11-58-22')
        # data1 is from inner imu
        # data2 is from outer imu
        corrimu_data1, bestpos_data1, spd_data1, ins_data1 = [], [], [], []
        corrimu_data2, bestpos_data2, spd_data2, ins_data2 = [], [], [], []
        for (ts, corrimu1), (ts, bestpos1), (ts, insspd1), (ts, inspvax1), (ts, corrimu2), (ts, bestpos2), (ts, insspd2), (ts, inspvax2), \
            in ds.fetch_aligned('/novatel_inner/novatel_data/corrimudata', '/novatel_inner/novatel_data/bestpos', '/novatel_inner/novatel_data/insspd', '/novatel_inner/novatel_data/inspvax',
                                '/novatel_data/corrimudata', '/novatel_data/bestpos', '/novatel_data/insspd', '/novatel_data/inspvax',
                                ts_begin=self.ts_begin, ts_end=self.ts_end):
            corrimu_data1.append(corrimu1)
            bestpos_data1.append(bestpos1)
            spd_data1.append(insspd1)
            ins_data1.append(inspvax1)
            corrimu_data2.append(corrimu2)
            bestpos_data2.append(bestpos2)
            spd_data2.append(insspd2)
            ins_data2.append(inspvax2)
        return corrimu_data1, bestpos_data1, spd_data1, ins_data1, corrimu_data2, bestpos_data2, spd_data2, ins_data2

    # parse pandar packet
    def parse_pandar(self):
        ds = Dataset.open(self.bag_name)
        pandar_left, pandar_right = [], []
        for (ts, left), (ts, right) in ds.fetch_aligned('/lidar_left/pandar_packets', '/lidar_right/pandar_packets', ts_begin=self.ts_begin, ts_end=self.ts_end):
            pandar_left.append(left)
            pandar_right.append(right)
        return pandar_left, pandar_right

    # parse vehiclebodyrotation
    def parse_rotation(self):
        ds = Dataset.open(self.bag_name)
        ins_data, rvb_data = [], []
        for (ts, rvb), (ts, ins) in ds.fetch_aligned('/novatel_data/vehiclebodyrotation', '/novatel_data/inspvax', ts_begin=self.ts_begin, ts_end=self.ts_end):
            rvb_data.append(rvb)
            ins_data.append(ins)
        return ins_data, rvb_data
