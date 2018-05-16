# compare imu result in different positions 
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
import sys
from imu_benchmark import IMUAnalyzer
 
# B2 outer imu
# bag_name = '2018-05-04-14-42-06'
# bag_name = '2018-04-30-16-49-43'
# bag_name = '2018-05-08-16-48-29'
# bag_name = '2018-05-08-16-48-29'
# bag_name = '2018-05-07-17-15-24'
# bag_name = '2018-05-09-17-14-03'
# bag_name = '2018-05-10-12-14-54'
# bag_name = '2018-05-09-18-52-08'
# bag_name = '2018-05-09-17-25-34'
# bag_name = '2018-05-10-14-45-41'
# bag_name = '2018-05-10-17-16-22'
# bag_name = '2018-05-10-18-21-11'
# bag_name = '/mnt/truenas/scratch/zhibei/b2_data_imu/2018-05-11-11-58-22'
# bag_name = '2018-05-11-11-08-22'
# bag_name = '2018-05-11-11-39-31'  # b1
# bag_name = '2018-05-14-11-02-11'
# bag_name = '2018-05-14-17-53-58'
# bag_name = '2018-05-14-18-25-21'
bag_name = '2018-05-15-19-05-16'
ts_begin = '10:00'
ts_end = '50:01'
info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
analyzer = IMUAnalyzer(info, 1)

data1, data2 = analyzer.data1, analyzer.data2
print len(data1['corrimu']['x_acc'])
print len(data2['corrimu']['x_acc'])

# def plot()
font = {'family': 'normal',
        'weight': 'bold',
        'size': 20}
plt.rc('font', **font)
# accelration
fig = plt.figure(1)
fig.suptitle('Acceleration in 3 axle', fontsize=30)
plt.subplot(311)
plt.plot(data2['corrimu']['x_acc'], 'b', label = 'acc_x in outer imu')
plt.plot(data1['corrimu']['x_acc'], 'r', linewidth =1,  label = 'acc_x in inner imu')

plt.xlabel('stamp')
plt.ylabel('m/s2')

plt.legend(loc='upper left')
plt.subplot(312)
plt.plot(data2['corrimu']['y_acc'], 'b', label = 'acc_y in outer imu')
plt.plot(data1['corrimu']['y_acc'], 'r', linewidth =1, label = 'acc_y in inner imu')
plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.subplot(313)
plt.plot(data2['corrimu']['z_acc'], 'b', label = 'acc_z in outer imu')
plt.plot(data1['corrimu']['z_acc'], 'r', linewidth =1, label = 'acc_z in inner imu')

plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.show()
    
# angular rate
fig = plt.figure(2)
fig.suptitle('Angular rate in 3 axle', fontsize=30)
plt.subplot(311)
plt.plot(data2['corrimu']['pitch_rate'], 'b', label = 'pitch_rate in outer imu')
plt.plot(data1['corrimu']['pitch_rate'], 'r', linewidth =1,  label = 'picth_rate in inner imu')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('rad/s')
plt.subplot(312)
plt.plot(data2['corrimu']['roll_rate'], 'b', label = 'roll_rate in outer imu')
plt.plot(data1['corrimu']['roll_rate'], 'r', linewidth =1, label = 'roll_rate in inner imu')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('rad/s')
plt.subplot(313)
plt.plot(data2['corrimu']['yaw_rate'], 'b', label = 'yaw_rate in outer imu')
plt.plot(data1['corrimu']['yaw_rate'], 'r', linewidth =1, label = 'yaw_rate in inner imu')

plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.show()


# position
fig = plt.figure(3)
fig.suptitle('Trajectory', fontsize=30)
plt.plot(data1['ins']['x'], data1['ins']['y'], 'r', label = 'inner imu trajectory')
plt.plot(data2['ins']['x'], data2['ins']['y'], 'b', label = 'outer imu trajectory')
plt.xlabel('East/m')
plt.ylabel('North/m')
plt.legend(loc='upper left')
plt.show()

# calibration result
fig = plt.figure(4)
fig.suptitle('Yaw angle calibration result', fontsize=30)
plt.subplot(411)
plt.plot(data1['ins']['yaw'], 'r', label = 'inner ins yaw')
plt.plot(data1['spd']['yaw'], 'b', label = 'inner spd yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(412)
diff = data1['ins']['yaw'] - data1['spd']['yaw']
plt.plot(diff, 'r', label = 'inspvax - insspd')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(413)
plt.plot(data2['ins']['yaw'], 'r', label = 'outer ins yaw')
plt.plot(data2['spd']['yaw'], 'b', label = 'outer spd yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(414)
diff = data2['ins']['yaw'] - data2['spd']['yaw']
plt.plot(diff, 'r', label = 'inspvax - insspd')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.show()


# yaw, insspd
fig = plt.figure(5)
fig.suptitle('Yaw from INSSPD', fontsize=30)
plt.subplot(411)
plt.plot(data1['spd']['yaw'], 'r', label = 'inner INSSPD yaw')
plt.plot(data2['spd']['yaw'], 'b', label = 'outer INSSPD yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')

plt.subplot(412)
plt.plot(data1['ins']['yaw'], 'r', label = 'inner inspvax yaw')
plt.plot(data2['ins']['yaw'], 'b', label = 'outer inspvax yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')

plt.subplot(413)
diff = data1['ins']['yaw'] - data2['ins']['yaw']
plt.plot(diff, 'r' , label = 'inner imu yaw - outer imu yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(414)
diff = data1['spd']['yaw'] - data2['spd']['yaw']
plt.plot(diff, 'r' , label = 'inner imu yaw - outer imu yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')

plt.show()

# leverarm result
# inner: -1.137, 2.78
# outer: -0.89, 3.4

# actual: 1.038, 2.76
# 0.838, 3.31
fig = plt.figure(6)
fig.suptitle('Leverarm', fontsize=30)
plt.subplot(211)
plt.plot(data1['offset']['x'], 'r', label = 'inner offset x')
plt.plot(data1['offset']['y'], 'b', label = 'inner offset y')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.subplot(212)
plt.plot(data2['offset']['x'], 'r', label = 'outer offset x')
plt.plot(data2['offset']['y'], 'b', label = 'outer offset y')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.show()

# lat, lon std
fig = plt.figure(7)
fig.suptitle('Lat/Lon Standard Deviation', fontsize=30)
plt.subplot(211)
plt.plot(data1['ins']['lat_std'], 'r', label = 'inner imu lat std')
plt.plot(data2['ins']['lat_std'], 'b', label = 'outer imu lat std')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.subplot(212)
plt.plot(data1['ins']['lon_std'], 'r', label = 'inner imu lat std')
plt.plot(data2['ins']['lon_std'], 'b', label = 'outer imu lat std')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.show()

fig = plt.figure(8)
fig.suptitle('Position Status', fontsize=30)
plt.plot(data1['ins']['status'], 'r', label = 'inner imu position status')
plt.plot(data2['ins']['status'], 'b', label = 'outer imu position status')
plt.xlabel('stamp')
plt.legend(loc='upper left')
plt.show()

fig = plt.figure(9)
fig.suptitle('Pitch & Roll', fontsize=30)
plt.subplot(211)
diff = np.mean(data1['ins']['pitch'] - data2['ins']['pitch'])

plt.plot(data1['ins']['pitch'], 'r', label = 'inner imu pitch')
plt.plot(data2['ins']['pitch'] + diff, 'b', label = 'outer imu pitch')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(212)
diff = np.mean(data1['ins']['roll'] - data2['ins']['roll'])
plt.plot(data1['ins']['roll'], 'r', label = 'inner imu roll')
plt.plot(data2['ins']['roll'] + diff, 'b', label = 'outer imu roll')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.show()