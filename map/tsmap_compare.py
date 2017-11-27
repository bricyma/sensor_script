import numpy as np
from tsmap import TSMap, Point3d
import rospy
from llh2enu.llh2enu_gps_transformer import *
import matplotlib.pyplot as plt


class Compare:
    def __init__(self):
        self.map_file = "/home/zhibei/Downloads/Beijing_Wuhudao_Forward_20171122_WHD.hdmap"
        with open(self.map_file, 'rb') as f:
            submap = f.read()
        self.map_handler = TSMap(submap)
        self.left_bounds, self.right_bounds = [], []

        self.old_map = {}
        self.old_map['x'] = []
        self.old_map['y'] = []

    def _get_bounds(self):
        print self.map_handler.bounds[0].points[0]
        for bound in self.map_handler.bounds:
            for point in bound.points:
                self.old_map['x'].append(point.x)
                self.old_map['y'].append(point.y)
        plt.plot(self.old_map['x'], self.old_map['y'], 'ro')
        plt.show()

if __name__ == '__main__':
    m = Compare()
    m._get_bounds()

