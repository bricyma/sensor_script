import rosbag
import matplotlib.pyplot as plt
import sys
import numpy as np
from parse import DataParser

bag_name = '2018-05-21-10-49-42'
bag_name = '2018-05-23-10-52-25'

ts_begin = '5:00'
ts_end = '100:01'
info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
parser = DataParser(info)

ins_data, rvb_data = parser.parse_rotation()

ins = {'z_angle': [], 'z_std': [], 'y_angle': [],
       'y_std': [], 'x_angle': [], 'x_std': []}

for msg in rvb_data:
    ins['z_angle'].append(msg.z_angle)
    ins['z_std'].append(msg.z_uncertainty)
    ins['y_angle'].append(msg.y_angle)
    ins['y_std'].append(msg.y_uncertainty)
    ins['x_angle'].append(msg.x_angle)
    ins['x_std'].append(msg.x_uncertainty)


def plot():
    plt.subplot(311)
    plt.plot(ins['x_angle'], 'r', label='x angle')
    plt.plot(ins['x_std'], 'b', label='x angle uncertainty')
    plt.legend(loc='upper left')
    plt.subplot(312)
    plt.plot(ins['y_angle'], 'r', label='y angle')
    plt.plot(ins['y_std'], 'b', label='y angle uncertainty')
    plt.legend(loc='upper left')
    plt.subplot(313)
    plt.plot(ins['z_angle'], 'r', label='z angle')
    plt.plot(ins['z_std'], 'b', label='z angle uncertainty')
    plt.legend(loc='upper left')
    plt.show()


plot()
