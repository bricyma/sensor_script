import rosbag
import matplotlib.pyplot as plt
import sys
import numpy as np
from parse import DataParser

bag_name = '2018-05-21-10-49-42'
ts_begin = '5:00'
ts_end = '100:01'
info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
parser = DataParser(info)

ins_data, rvb_data = parser.parse_rotation()

ins = {'z_angle': [], 'z_std': []}

for msg in ins_data:
    ins['z_angle'].append(msg.z_angle)
    ins['z_std'].append(msg.z_uncertainty)
    
def plot():
    plt.plot(ins['z_angle'], 'r', label='z angle')
    plt.plot(ins['z_std'], 'b', label='z angle uncertainty')
    plt.legend(loc='upper left')
    plt.show()
plot()
