import os
import sys
from collections import defaultdict


class RecordCheck:
    def __init__(self):
        self.rosbagpath = sys.argv[1]  # /mnt/scratch/datasets/2017-05-22/20170522131932
        self.rosinfo_file = self.rosbagpath[-14:] + '.txt'  #20170522131932.txt
        self.config_file = 'heartbeat_config.csv'
        self.input_file = ''
        self.topic_flag = 0
        self.duration = 0
        self.topic_num = 0
        self.topic_msg_dict = {}  # topic, real msg number in rosbag
        self.topic_info_dict = {}  # topic, expected msg number
        self.topic_res = {}  # topic, whether or not msg number meets the requirement
        self.config = defaultdict(dict)
        self.topics = []
        self.sensor_dict = {}  # topic: sensor
        self.delta = 0.1
        self.signal_dict = {1: 'ok', 2: 'higher', -1: 'lower'}  # 1, meets the requirement; 2, higher than requirement; 3, lower than requirement

    def get_topic(self):
        for line in self.input_file:
            l = line.split(':')
            if l[0] == 'duration':
                # l: 'duration:    4:54s (294s)'
                print line.split('(')[1][:-3]
                self.duration = float(line.split('(')[1][:-3])

                # self.duration = float(l[1].strip()[:-1])
            if l[0] == 'topics':
                self.topic_flag = 1
            if self.topic_flag:
                if self.topic_num == 0:
                    topic_msg = l[1].strip()
                else:
                    topic_msg = l[0].strip()

                topic_array = topic_msg.split(' ')
                topic = topic_array[0]
                topic_msg_num = 0
                for i in range(1, len(topic_array)):
                    if topic_array[i] != '':
                        topic_msg_num = int(topic_array[i])
                        break
                self.topic_msg_dict[topic] = topic_msg_num
                self.topic_num += 1

    def read_config(self):
        with open(self.config_file) as f:
            for line in f:
                fields = line.strip().split(",")
                sensor = fields[0].strip()
                topic = fields[1].strip()
                check_flag = True if fields[3] == 'T' else False
                self.config[sensor][topic] = (int(fields[2]), check_flag)
                self.sensor_dict[topic] = sensor
                self.topic_info_dict[topic] = int(fields[2])
                self.topics.append((sensor, topic))

    def check_msg(self):
        # sensor: 'Lidar'
        for sensor in self.config:
            # topic: 'velodyne/packet'
            for topic in self.config[sensor]:
                if self.config[sensor][topic][1]:
                    flag = self.compare(self.topic_msg_dict[topic], self.topic_info_dict[topic])
                    self.topic_res[topic] = flag

    def compare(self, cur, frequency):
        # cur: current; req: required
        # 1, meets the requirement; 2, higher than requirement; 3, lower than requirement
        if self.duration * frequency * (1 - self.delta) < cur < self.duration * frequency * (1 + self.delta):
            return 1
        elif cur > self.duration * frequency * (1 + self.delta):
            return 2
        else:
            return -1

    def show(self):
        # for topic in self.topic_res:
        #     print "%-10s %-50s %s" % (self.sensor_dict[topic], topic, self.signal_dict[self.topic_res[topic]])

        # print in order of topics
        for topic in self.topics:
            if topic in self.topic_res:
                print "%-10s %-50s %s" % (self.sensor_dict[topic], topic, self.signal_dict[self.topic_res[topic]])

    # create rosbag info txt
    def createtxt(self):
        print 'create rosbag info txt'
        # os.system('rosbag info /home/zhibei/workspace/rosbag/2017-05-22-14-23-01.bag > b.txt')
        os.system('rosbag info ' + self.rosbagpath + '/*.bag >' + self.rosinfo_file)
        self.input_file = open(self.rosinfo_file, 'r')

    def run(self):
        self.createtxt()
        self.get_topic()
        self.read_config()
        self.check_msg()
        self.show()

if __name__ == '__main__':
    check = RecordCheck()
    check.run()
