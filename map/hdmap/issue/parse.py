import sys
import os
from dataset_store import Dataset

# input: bag_name, ts_begin, ts_end
# output: error data, localization pose, inspvax and bestvel 
class DataParser:
    def __init__(self, info):
        print info
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

