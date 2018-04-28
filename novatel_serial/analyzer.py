import rosbag
import matplotlib.pyplot as plt
import sys
import numpy as np

bagname = sys.argv[1]
bag = rosbag.Bag(bagname)

ins = {'t_ros': [], 't_gps': []}


def read_data():
    cnt = 0
    for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
        cnt += 1
        if cnt > 15000:
            break
        pc_time = float(str(msg.header2.stamp.secs)) \
            + float(str(msg.header2.stamp.nsecs)) * \
            1e-9  # epoch second
        novatel_time = 315964800 + msg.header.gps_week * 7 * 24 * 60 * 60 \
            + float(msg.header.gps_week_seconds) / \
            1000 - 18  # GPS time to epoch second
        ins['t_ros'].append(pc_time)
        ins['t_gps'].append(novatel_time)

    ins['t_ros'], ins['t_gps'] = np.array(ins['t_ros']), np.array(ins['t_gps'])
    bag.close()


def plot():
    font = {'family': 'normal', 'weight': 'bold', 'size': 30}
    plt.rc('font', **font)
    diff_ros = ins['t_ros'][1:] - ins['t_ros'][:-1]
    diff_gps = ins['t_gps'][1:] - ins['t_gps'][:-1]
    plt.plot(diff_ros, 'r', label='inspvax t_ros interval')
    plt.plot(diff_gps, 'g', linewidth=3,  label='inspvax t_gps interval')
    plt.legend(loc='upper left')
    plt.xlabel('stamp')
    plt.ylabel('time(s)')
    plt.show()


read_data()
plot()
