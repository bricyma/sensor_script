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

esr_yawrate = []
veh_yawrate = []
veh_twist_yawrate = []

for topic, msg, t in bag.read_messages(topics=['/as_tx/radar_status']):
    esr_yawrate.append(msg.ESRyawRate)

flag = 1
for topic, msg, t in bag.read_messages(topics=['/as_rx/vehicle_motion']):
    if flag:
        flag = 0
        veh_yawrate.append(msg.twist.angular.z * 180 / m.pi)
    else:
        flag = 1

flag = 1
for topic, msg, t in bag.read_messages(topics=['/vehicle/twist']):
    if flag:
        flag = 0
        veh_twist_yawrate.append(msg.twist.angular.z * 180 / m.pi)
    else:
        flag = 1

bag.close()
plt.plot(esr_yawrate, 'b', label='/as_tx/radar_status')
plt.plot(veh_yawrate, 'g', label='/as_rx/vehicle_motion')
plt.plot(veh_twist_yawrate, 'r', label='/vehicle/twist')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('yaw rate')
plt.title("YAW Rate between ESR and Vehicle")
plt.show()



