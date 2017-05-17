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

# yr = yaw rate
ins_yr = []
vehicle_yr = []
vehicle_imu_yr = []

ins_angle_x = []
vehicle_imu_angle_x = []
ins_angle_y = []
vehicle_imu_angle_y = []

ins_acc_x = []
ins_acc_y = []
ins_acc_z = []

imu_acc_x = []
imu_acc_y = []
imu_acc_z = []

veh_acc_x = []
veh_acc_y = []
veh_acc_z = []

veh_vel = []
veh_vel_acc = []

imu = []
ins = []
veh = []
gyro_scale_factor = 0.1/(3600.0*256.0)
acc_scale_factor = 0.05/(2**15)
imu_rate = 200


def read_data():
    for topic, msg, t in bag.read_messages(topics=['/novatel_data/rawimu']):
        imu_acc_x.append(msg.x_accel*acc_scale_factor*imu_rate)
        imu_acc_y.append(-msg.y_accel*acc_scale_factor*imu_rate)
        imu_acc_z.append(msg.z_accel*acc_scale_factor*imu_rate)

    for topic, msg, t in bag.read_messages(topics=['/novatel_data/corrimudata']):
        ins_acc_x.append(msg.x_accel*imu_rate)
        ins_acc_y.append(msg.y_accel*imu_rate)
        ins_acc_z.append(msg.z_accel*imu_rate)

    flag = 1
    for topic, msg, t in bag.read_messages(topics=['/vehicle/imu/data_raw']):
        if flag:
            flag = 0
            veh_acc_x.append(msg.linear_acceleration.x)
            veh_acc_y.append(msg.linear_acceleration.y)
            veh_acc_z.append(msg.linear_acceleration.z)
        else:
            flag = 1

    before = 0
    for topic, msg, t in bag.read_messages(topics=['/vehicle/twist']):
        veh_vel.append(msg.twist.linear.x)
        now = msg.twist.linear.x
        x_acc = (now-before)/0.02
        veh_vel_acc.append(x_acc)
        before = now

    bag.close()

    imu.append(imu_acc_x)
    imu.append(imu_acc_y)
    imu.append(imu_acc_z)
    ins.append(ins_acc_x)
    ins.append(ins_acc_y)
    ins.append(ins_acc_z)
    veh.append(veh_acc_x)
    veh.append(veh_acc_y)
    veh.append(veh_acc_z)


def plot():
    for i in range(0, 3):
        plt.plot(veh_vel_acc, 'g', label='veh acc from twist')
        plt.plot(ins[i], 'b', label='corrimudata')
        # plt.plot(imu[i], 'g', label='rawimu')
        if i == 0:
            plt.plot(veh[i], 'r', label='negative vehicle imu')
            plt.title("Linear acceleration on X-axle between CORRIMUDATA and RAWIMU")
        if i == 1:
            plt.plot(veh[i], 'r', label='vehicle imu')
            plt.title("Linear acceleration on Y-axle between CORRIMUDATA and RAWIMU")
        if i == 2:
            plt.plot(veh[i], 'r', label='vehicle imu')
            plt.title("Linear acceleration on Z-axle between CORRIMUDATA and RAWIMU")
        plt.legend(loc='upper left')
        plt.xlabel('stamp')
        plt.ylabel('linear acceleration (m/s^2)')
        plt.show()


read_data()
plot()
