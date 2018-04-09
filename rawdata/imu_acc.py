import rosbag
import matplotlib.pyplot as plt
import sys
import numpy as np

bagname = ''
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)

ins = {'acc_x': [], 'acc_y': [], 'acc_z': [],
       'ang_x': [], 'ang_y': [], 'ang_z': []}
imu = {'acc_x': [], 'acc_y': [], 'acc_z': [],
       'ang_x': [], 'ang_y': [], 'ang_z': []}

# IMU: IMU-IGM-S1
gyro_scale_factor = 2**-21
acc_scale_factor = 2**-22
imu_rate = 125


def read_data():
    for topic, msg, t in bag.read_messages(topics=['/novatel_data/rawimu']):
        imu['acc_x'].append(msg.x_accel * acc_scale_factor * imu_rate)
        imu['acc_y'].append(-msg.y_accel * acc_scale_factor * imu_rate)
        imu['acc_z'].append(msg.z_accel * acc_scale_factor * imu_rate)
        imu['ang_x'].append(msg.pitch_rate * gyro_scale_factor)
        imu['ang_y'].append(-msg.roll_rate * gyro_scale_factor)
        imu['ang_z'].append(msg.yaw_rate * gyro_scale_factor)

    for topic, msg, t in bag.read_messages(topics=['/novatel_data/corrimudata']):
        ins['acc_x'].append(msg.x_accel * imu_rate)
        ins['acc_y'].append(msg.y_accel * imu_rate)
        ins['acc_z'].append(msg.z_accel * imu_rate)
        ins['ang_x'].append(msg.pitch_rate * imu_rate)
        ins['ang_y'].append(msg.roll_rate * imu_rate)
        ins['ang_z'].append(msg.yaw_rate * imu_rate)
    bag.close()


def plot():
    plt.subplot(311)
    plt.plot(imu['ang_x'], 'r', label='raw imu ang_x')
    plt.plot(ins['ang_x'], 'g', label='corrected ang_x')
    plt.legend(loc='upper left')
    plt.subplot(312)
    plt.plot(imu['ang_y'], 'r', label='raw imu ang_y')
    plt.plot(ins['ang_y'], 'g', label='corrected ang_y')
    plt.legend(loc='upper left')
    plt.subplot(313)
    plt.plot(imu['ang_z'], 'r', label='raw imu ang_z')
    plt.plot(ins['ang_z'], 'g', label='corrected ang_z')
    plt.legend(loc='upper left')
    plt.show()

    plt.subplot(311)
    plt.plot(imu['acc_x'], 'r', label='raw imu acc_x')
    plt.plot(ins['acc_x'], 'g', label='corrected acc_x')
    plt.legend(loc='upper left')
    plt.subplot(312)
    plt.plot(imu['acc_y'], 'r', label='raw imu acc_y')
    plt.plot(ins['acc_y'], 'g', label='corrected acc_y')
    plt.legend(loc='upper left')
    plt.subplot(313)
    plt.plot(imu['acc_z'], 'r', label='raw imu acc_z')
    plt.plot(ins['acc_z'], 'g', label='corrected acc_z')
    plt.legend(loc='upper left')
    plt.show()
read_data()
plot()
