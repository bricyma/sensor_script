import numpy as np
from tsmap import TSMap, Point3d
import rospy
from llh2enu.llh2enu_gps_transformer import *
import matplotlib.pyplot as plt
import struct

class Compare:
    def __init__(self, id, map_file):
        with open(map_file, 'rb') as f:
            submap = f.read()
        self.map_handler = TSMap(submap)
        # get the start point
        f = open(map_file, 'rb')
        f.read(12)
        version_num = struct.unpack('i', f.read(4))[0]
        msg = f.read(72)
        result = struct.unpack('ddddddddd', msg)
        self.start_point = result[:3]
        self.base_latlon = struct.unpack('dd', f.read(16))
        self.id = id
        self.map = {}
        self.map['x'] = []
        self.map['y'] = []

    def _get_bounds(self):
        print self.map_handler.bounds[0].points[0]
        for bound in self.map_handler.bounds:
            for point in bound.points:
                self.map['x'].append(point.x)
                self.map['y'].append(point.y)
        self.map['x'] = np.array(self.map['x'])
        self.map['y'] = np.array(self.map['y'])
        self.map['x'] = self.map['x'] - self.start_point[0]
        self.map['y'] = self.map['y'] - self.start_point[1]

        plt.xlim([-3500, 500])
        plt.ylim([-3500, 500])

        if self.id == 0:
            plt.plot(self.map['x'], self.map['y'], 'ro')
        elif self.id == 1:
            plt.plot(self.map['x'], self.map['y'], 'bo')
        else:
            plt.plot(self.map['x'], self.map['y'], 'go')


if __name__ == '__main__':
    m_whd = Compare(
        0, "/home/zhibei/Downloads/Beijing_Wuhudao_Forward_20171122_WHD.hdmap")
    m_whd._get_bounds()
    m_cfd = Compare(
        # 1, "/home/zhibei/Downloads/Beijing_Wuhudao_Forward_20171122_CFD.hdmap")
        1, "/home/zhibei/Downloads/Beijing_Wuhudao_Forward_TestOnly_20171128.hdmap")
    m_cfd._get_bounds()
    m_cfd = Compare(
        2, "/home/zhibei/.octopus/maps/Wuhudao_forward_0912.hdmap")
    m_cfd._get_bounds()
    plt.show()
