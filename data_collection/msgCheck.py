import os
from collections import defaultdict


class RecordCheck:
    def __init__(self):
        self.bag_file = 'b.txt'
        self.config_file = 'test.csv'
        self.input_file = open(self.bag_file, 'r')
        self.topic_flag = 0
        self.duration = 0
        self.topic_num = 0
        self.topic_msg_dict = {}  # topic, real msg number in rosbag
        self.topic_info_dict = {}  # topic, expected msg number
        self.topic_res = {} # topic, whether or not msg number meets the requirement
        self.config = defaultdict(dict)
        self.topics = []
        self.sensor_dict = {}  # topic: sensor
        self.delta = 0.1

    def get_topic(self):
        for line in self.input_file:
            l = line.split(':')
            if l[0] == 'duration':
                self.duration = float(l[1].strip()[:-1])
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
        if self.duration * frequency * (1 - self.delta) < cur < self.duration * frequency * (1 + self.delta):
            return 1
        else:
            return 0

    def show(self):
        for topic in self.topic_res:
            print topic, self.topic_res[topic], self.sensor_dict[topic]

    # create rosbag info txt
    def createtxt(self):
        os.system('rosbag info 2017-05-26-00-02-25.bag > aaac.txt')

    def run(self):
        self.createtxt()
        self.get_topic()
        self.read_config()
        self.check_msg()
        self.show()

if __name__ == '__main__':
    check = RecordCheck()
    check.run()
