import rosbag
import matplotlib.pyplot as plt
import sys
import numpy as np

bagname = sys.argv[1]
bag = rosbag.Bag(bagname)

ins = {'t_ros': [], 't_gps': []}


def read_data():
    for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvas']):
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
    diff_ros = ins['t_ros'][1:] - ins['t_ros'][:-1]
    diff_gps = ins['t_gps'][1:] - ins['t_gps'][:-1]
    plt.plot(diff_ros, 'r', label='t_ros diff')
    plt.plot(diff_gps, 'g', label='t_gps diff')
    plt.legend(loc='upper left')
    plt.show()


read_data()
plot()
