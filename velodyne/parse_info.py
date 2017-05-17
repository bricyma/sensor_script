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
        total = x[1]
        msg = total.msg
        l = msg.split(' ')
        # print l[0], l[1], l[2]
        if num > 100:
            if float(l[0])>=171 or int(l[0])==0:
                print l[0], l[1]
        if num > 40000:
            index.append(float(l[0]))
            rotation.append(float(l[1]))
            if float(l[2]) > 0:
                diff.append(float(l[2]))
        num += 1
        if num > 60000:
            break
    bag.close()


def plot():
    plt.plot(diff, 'r')
    plt.show()


if __name__ == '__main__':

    read_data()
    print np.mean(diff)
    plot()
