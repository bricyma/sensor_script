import sys, os
from dataset_store import Dataset


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
		for (ts, error), (ts_gps, gps), (ts_gps, bestvel) in ds.fetch_aligned('/error_monitor/error_info','/novatel_data/inspvax', '/novatel_data/bestvel', 
        	ts_begin=self.ts_begin, ts_end=self.ts_end):
   	 		error_data.append(error)
			gps_data.append(gps)
			bestvel_data.append(bestvel)
		return error_data, gps_data, bestvel_data