import rosbag
import math as m
import numpy as np
import matplotlib.pyplot as plt
import sys

bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)

index = []
rotation = []
diff = []



def read_data():
    num = 0
    for x in bag.read_messages(topics=['/rosout']):
        total_msg = x[1]
        num += 1
        if num % 10 != 0 or total_msg.name.strip() != '/velodyne_nodelet_manager':
            continue
        msg = total_msg.msg

        l = msg.split(' ')
        if float(l[2]) > 0:
            diff.append(float(l[2]))
    bag.close()


def plot():
    plt.plot(diff, 'r')
    plt.show()


if __name__ == '__main__':

    read_data()
    print np.mean(diff)
    plot()
