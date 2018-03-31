# #!/usr/bin/env python

# analyze the relationship between postion deviation and yaw offset
from parse import DataParser
from analyzer import YawAnalyzer
import matplotlib.pyplot as plt
import numpy as np

class ErrorAnalyzer:
	def __init__(self, info):
		parser = DataParser(info)
		self.file_name = info['bag_name']
		self.ts_begin = info['ts_begin']
		self.ts_end = info['ts_end']
		# get data from dataset
		error, ins_data, bestvel_data = parser.parse()
		# warp gps data
		gps_data = self.gps_warp(ins_data, bestvel_data)
		error_data = self.error_warp(error)
		# parse gps data to get the yaw difference
		yaw_analyzer = YawAnalyzer(error_data, gps_data)
		# gps_data_window={'d_yaw1':[], 'd_yaw2':[], 'd_yaw3':[]}
		# d_yaw1: inspvax - local_yaw
		# d_yaw2: bestvel - local_yaw
		# d_yaw3: inspvax - bestvel
		errow_window, error_no_window, gps_data_window, gps_data_no_window = yaw_analyzer.parse()  # yaw difference
	

		self.plot(errow_window, error_no_window, gps_data_window, gps_data_no_window)

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
			data.append(msg.dis_to_lane_center)
			# data.append(msg.crosstrack_error)
		return data

	def plot(self, error_window, error_no_window, gps_data_window, gps_data_no_window):
		fig = plt.figure()
		fig.suptitle(self.file_name + ' ' + self.ts_begin + '~' + self.ts_end, fontsize=20)
		Len = len(gps_data_window['d_yaw1'])
		plt.figure(1)

		plt.subplot(411)
		plt.plot(gps_data_window['d_yaw2'], 'r.', label='window: bestvel - local_yaw')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.plot([np.mean(gps_data_no_window['d_yaw2'])]*Len, 'b', linewidth=3,  label='mean')
		plt.legend(loc=2, prop={'size': 9})
		plt.subplot(412)
		plt.plot(gps_data_no_window['d_yaw2'], 'r.', label='no window: bestvel - local_yaw')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.plot([np.mean(gps_data_no_window['d_yaw2'])]*Len, 'b', linewidth=3,  label='mean')
		print 'mean of (bestvel-local_yaw): ', np.mean(gps_data_no_window['d_yaw2'])
		plt.legend(loc=2, prop={'size': 9})
		plt.subplot(413)
		plt.plot(gps_data_window['d_yaw3'], 'r.', label='window: inspvax - bestvel')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.legend(loc=2, prop={'size': 9})
		plt.subplot(414)
		plt.plot(gps_data_no_window['d_yaw3'], 'r.', label='no window: inspvax - bestvel')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.legend(loc=2, prop={'size': 9})

		plt.figure(2)
		plt.subplot(411)
		plt.plot(gps_data_window['d_yaw1'], 'r.', label='window: inspvax - local_yaw')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.legend(loc=2, prop={'size': 9})
		plt.subplot(412)
		plt.plot(gps_data_no_window['d_yaw1'], 'r.', label='no window: inspvax - local_yaw')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.legend(loc=2, prop={'size': 9})
		plt.subplot(413)
		error_window = np.array(error_window)
		error_window[abs(error_window)>1] = 0
		plt.plot(error_window, 'r.', label='window: error_data')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.legend(loc=2, prop={'size': 9})
		plt.subplot(414)
		error_no_window = np.array(error_no_window)
		error_no_window[abs(error_no_window)>1] = 0
		plt.plot(error_no_window, 'r.', label='no window: error_data')
		plt.plot([0]*Len, 'g', linewidth=3,  label='0')
		plt.legend(loc=2, prop={'size': 9})
		plt.show()

if __name__ == '__main__':
	# bag_name = '2018-03-24-17-52-31' # no error_info 
	# bag_name = '2018-03-25-16-35-56'
	# bag_name = '2018-03-25-18-20-44'
	# bag_name = '2018-03-25-19-19-41'
	# bag_name = '2018-03-27-13-46-05'
	# bag_name = '2018-03-27-16-03-26'	
	bag_name = '2018-03-27-21-40-24'
	# bag_name = '2018-03-27-22-47-57'
	# bag_name = '2018-03-28-16-55-36'
	# bag_name = '2018-03-28-18-18-45'
	# bag_name = '2018-03-29-14-28-56'
	# bag_name = '2018-03-29-15-51-57'
	# bag_name = '2018-03-29-16-52-35'

	ts_begin = '0:00'
	ts_end = '60:00'
	info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
	analyzer = ErrorAnalyzer(info)
