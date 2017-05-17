import rosbag
import math as m
import matplotlib.pyplot as plt
import sys
import numpy as np
bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)
sequence = 0
index = []

esr_status = []
for topic, msg, t in bag.read_messages(topics=['/as_tx/radar_tracks']):
    print len(msg.tracks)

bag.close()
# plt.plot(esr_status, 'b', label='/parsed_tx/radartrack')
# plt.legend(loc='upper left')
# plt.xlabel('stamp')
# plt.ylabel('yaw rate')
# plt.title("YAW Rate between ESR and Vehicle")
plt.show()



