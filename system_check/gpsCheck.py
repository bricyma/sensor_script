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
# gps position from /novatel_data/bestpos
lat_best = []
lng_best = []
alt_best = []

# gps position from /novatel_data/inspvax
lat_ins = []
lng_ins = []
alt_ins = []

ori = []

# yr = yaw rate
ins_yr = []
vehicle_yr = []
vehicle_imu_yr = []
ins_angle_x = []
vehicle_imu_angle_x = []
ins_angle_y = []
vehicle_imu_angle_y = []
for topic, msg, t in bag.read_messages(topics=['/novatel_data/bestpos']):
    lat_best.append(msg.latitude)
    lng_best.append(msg.longitude)
    alt_best.append(msg.altitude)
    index.append(sequence)
    sequence += 1

for topic, msg, t in bag.read_messages(topics=['/novatel_data/corrimudata']):
    ins_yr.append(msg.yaw_rate*200)
    ins_angle_x.append(msg.pitch_rate*200)
    ins_angle_y.append(msg.roll_rate*200)

flag = 1
for topic, msg, t in bag.read_messages(topics=['/vehicle/imu/data_raw']):
    if flag:
        flag = 0
        vehicle_imu_yr.append(msg.angular_velocity.z)
        vehicle_imu_angle_x.append(msg.angular_velocity.x)
        vehicle_imu_angle_y.append(msg.angular_velocity.y)
    else:
        flag = 1



for topic, msg, t in bag.read_messages(topics=['/vehicle/twist']):
    vehicle_yr.append(msg.twist.angular.z)

bag.close()
# plt.ylim([-180,180])
# plt.plot(lat_best, 'b', label='bestpos latitude')
# plt.plot(lng_best, 'r', label='bestpos longitude')
# plt.plot(lat_ins, 'y', label='latitude latitude')
# plt.plot(lng_ins, 'g', label='longitude longitude')

# plt.plot(ori, 'r', label='azimuth')

# for compare imu angular velocity
# plt.plot(ins_yr, 'b', label='corrimudata')
# plt.plot(vehicle_yr, 'r', label='vehicle twist')
# plt.plot(vehicle_imu_yr, 'g', label='vehicle imu')

plt.plot(ins_angle_y, 'b', label='corrimudata')
plt.plot(vehicle_imu_angle_y, 'g', label='vehicle imu')





plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('yaw rate (rad/s)')

# plt.plot(lat_std)
# plt.plot(lng_std)
# plt.plot(alt_std)
plt.title("Yaw rate from corrimudata, twist and vehicle/imu")

plt.show()


