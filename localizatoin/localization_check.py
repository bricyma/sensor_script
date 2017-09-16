import rosbag
import numpy as np
from matplotlib import pyplot as plt

class LocCheck:
    def __init__(self, bagname):
        self.data = {}
        self.bag = rosbag.Bag(bagname)
        self.data['x'] = []  # gnss pose x
        self.data['y'] = []
        self.data['x2'] = [] # image_localization_fusion pose x
        self.data['y2'] = []
        self.data['yaw'] = []
        self.diff_en = ()
        self.diff_xy = ()

    def import_bag(self):
        count = 1000
        i = 0
        for topic, msg, t in self.bag.read_messages(topics=['/image_localization_fusion/pose']):
            self.data['x2'].append(msg.pose.position.x)
            self.data['y2'].append(msg.pose.position.y)
            i += 1
            if i > count:
                break

        i = 0
        for topic, msg, t in self.bag.read_messages(topics=['/gnss/pose']):
            self.data['x'].append(msg.pose.position.x)
            self.data['y'].append(msg.pose.position.y)
            i += 1
            if i > count:
                break
        i = 0
        for topic, msg, t in self.bag.read_messages(topics=['/novatel_data/inspvax']):
            self.data['yaw'].append(msg.azimuth)
            i += 1
            if i > count:
                break

        self.data['x'] = np.array(self.data['x'])
        self.data['x2'] = np.array(self.data['x2'])
        self.data['y'] = np.array(self.data['y'])
        self.data['y2'] = np.array(self.data['y2'])
        self.data['yaw'] = self.angle2rad(np.array(self.data['yaw']))

    def angle2rad(self, angle):
        return angle / 180 * np.pi


    def calculate(self):
        length = len(self.data['yaw'])

        self.diff_en = self.data['x2'] - self.data['x'], self.data['y2'] - self.data['y']

        x = self.diff_en[0]
        y = self.diff_en[1]
        self.diff_xy = -x * np.cos(self.data['yaw']) + y * np.sin(self.data['yaw']), \
                   x * np.sin(self.data['yaw']) + y * np.cos(self.data['yaw'])

    def plot(self):
        plt.plot(self.diff_xy[0], label='x')
        plt.plot(self.diff_xy[1], label='y')
        plt.legend(loc='upper left')

        plt.show()

    def run(self):
        self.import_bag()
        self.calculate()
        self.plot()
        print 'aa'


if __name__ == '__main__':
    bag = '2017-09-16-18-25-22.bag'
    prefix = '/home/zhibei/workspace/rosbag/'
    check = LocCheck(prefix+bag)
    check.run()