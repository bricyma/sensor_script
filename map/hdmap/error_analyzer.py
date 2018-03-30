# #!/usr/bin/env python

# analyze the relationship between postion deviation and yaw offset
from parse import DataParser
from analyzer import YawAnalyzer
import matplotlib.pyplot as plt

class ErrorAnalyzer:
	def __init__(self, info):
		parser = DataParser(info)
		# get data from dataset
		error, ins_data, bestvel_data = parser.parse()
		# warp gps data
		gps_data = self.gps_warp(ins_data, bestvel_data)

		# parse gps data to get the yaw difference
		yaw_analyzer = YawAnalyzer(gps_data)
		# gps_data_window={'d_yaw1':[], 'd_yaw2':[], 'd_yaw3':[]}
		# d_yaw1: inspvax - local_yaw
		# d_yaw2: bestvel - local_yaw
		# d_yaw3: inspvax - bestvel
		gps_data_window, gps_data_no_window = yaw_analyzer.parse()  # yaw difference
		error_data = self.error_warp(error)
		self.plot(gps_data_window, error_data)

	def gps_warp(self, ins_data, bestvel_data):
		data = {'lat':[], 'lon':[], 'azimuth':[], 'trk_gnd':[]}
		for msg in ins_data:
			data['lat'].append(msg.latitude)
			data['lon'].append(msg.longitude)
			data['azimuth'].append(msg.azimuth)
		for msg in bestvel_data:
			data['trk_gnd'].append(msg.trk_gnd)
		return data

	def error_warp(self, error):
		data = []
		for msg in error:
			data.append(msg.crosstrack_error)
		return data

	def plot(self, gps_data_window, error_data):
		plt.subplot(211)
		plt.plot(gps_data_window['d_yaw1'], 'r.', label='inspvax - local_yaw')
		plt.subplot(212)
		plt.plot(error_data, 'r.', label='error_data')
		plt.legend(loc=2, prop={'size': 9})
		plt.show()

if __name__ == '__main__':
	bag_name = '2018-03-29-15-51-57'
	ts_begin = '0:00'
	ts_end = '50:00'
	info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
	analyzer = ErrorAnalyzer(info)
