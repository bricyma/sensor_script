# compare imu result in different positions 
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
import sys
from gnss_imu_benchmark import IMUAnalyzer
 
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
# bag_name = '2018-05-15-19-05-16'
# bag_name = '2018-05-16-10-54-03'
# bag_name = '2018-04-09-14-40-14'
# bag_name = '2018-04-06-11-39-06'
# bag_name = '2018-04-04-14-30-09'
# bag_name = '2018-04-10-10-48-30'
# bag_name = '2018-05-16-13-02-16'
# bag_name = '2018-05-16-14-38-07'
# bag_name = '2018-05-16-15-43-19'
# bag_name = '2018-05-16-10-54-03'
# bag_name = '2018-05-17-11-57-48'
# bag_name = '2018-05-17-10-50-21'
# bag_name = '2018-05-17-14-39-13'


# b1
bag_name = '2018-05-18-11-29-26'
ts_begin = '15:00'
ts_end = '100:01'
info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
analyzer = IMUAnalyzer(info, 1)

data1, data1 = analyzer.data1, analyzer.data1
print len(data1['corrimu']['x_acc'])
print len(data1['corrimu']['x_acc'])

# def plot()
font = {'family': 'normal',
        'weight': 'bold',
        'size': 20}
plt.rc('font', **font)
# accelration
fig = plt.figure(1)
fig.suptitle('Acceleration in 3 axle', fontsize=30)
plt.subplot(311)
plt.plot(data1['corrimu']['x_acc'], 'b', label = 'acc_x in outer imu')
plt.plot(data1['corrimu']['x_acc'], 'r', linewidth =1,  label = 'acc_x in inner imu')

plt.xlabel('stamp')
plt.ylabel('m/s2')

plt.legend(loc='upper left')
plt.subplot(312)
plt.plot(data1['corrimu']['y_acc'], 'b', label = 'acc_y in outer imu')
plt.plot(data1['corrimu']['y_acc'], 'r', linewidth =1, label = 'acc_y in inner imu')
plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.subplot(313)
plt.plot(data1['corrimu']['z_acc'], 'b', label = 'acc_z in outer imu')
plt.plot(data1['corrimu']['z_acc'], 'r', linewidth =1, label = 'acc_z in inner imu')

plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.show()
    
# angular rate
fig = plt.figure(2)
fig.suptitle('Angular rate in 3 axle', fontsize=30)
plt.subplot(311)
plt.plot(data1['corrimu']['pitch_rate'], 'b', label = 'pitch_rate in outer imu')
plt.plot(data1['corrimu']['pitch_rate'], 'r', linewidth =1,  label = 'picth_rate in inner imu')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('rad/s')
plt.subplot(312)
plt.plot(data1['corrimu']['roll_rate'], 'b', label = 'roll_rate in outer imu')
plt.plot(data1['corrimu']['roll_rate'], 'r', linewidth =1, label = 'roll_rate in inner imu')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('rad/s')
plt.subplot(313)
# plt.plot(data1['corrimu']['yaw_rate'], 'b', label = 'yaw_rate in outer imu')
# plt.plot(data1['corrimu']['yaw_rate'], 'r', linewidth =1, label = 'yaw_rate in inner imu')

diff1 = data1['corrimu']['yaw_rate'][1:] - data1['corrimu']['yaw_rate'][:-1]
diff2 = data1['corrimu']['yaw_rate'][1:] - data1['corrimu']['yaw_rate'][:-1]

print 'std of yaw rate diff: ',  np.std(diff1), np.std(diff2)

plt.plot(diff2, 'b', label = 'yaw_rate change in outer imu')
plt.plot(diff1, 'r', linewidth =1, label = 'yaw_rate change in inner imu')



plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.show()


# position
fig = plt.figure(3)
fig.suptitle('Trajectory', fontsize=30)
plt.plot(data1['ins']['x'], data1['ins']['y'], 'r', label = 'inner imu trajectory')
plt.plot(data1['ins']['x'], data1['ins']['y'], 'b', label = 'outer imu trajectory')
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
plt.plot(data1['ins']['yaw'], 'r', label = 'outer ins yaw')
plt.plot(data1['spd']['yaw'], 'b', label = 'outer spd yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(414)
diff = data1['ins']['yaw'] - data1['spd']['yaw']
plt.plot(diff, 'r', label = 'inspvax - insspd')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.show()


fig = plt.figure(5)
fig.suptitle('Leverarm', fontsize=30)
plt.subplot(211)
plt.plot(data1['pos']['x'], 'r', label = 'pos offset x between inspvax and lane center')
plt.plot(data1['pos']['y'], 'b', label = 'pos offset y between inspvax and lane center')
plt.legend(loc='upper left')
plt.show()

# leverarm result
# inner: -1.037, 2.78
# outer: -0.89, 3.4

# actual: 1.038, 2.76
# 0.838, 3.31
fig = plt.figure(6)
fig.suptitle('Leverarm', fontsize=30)
plt.subplot(311)
plt.plot(data1['offset']['x'], 'r', label = 'leverarm offset x')
plt.plot(data1['offset']['y'], 'b', label = 'leverarm offset y')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.subplot(312)
plt.plot(data1['ins']['yaw'], 'r', label = 'ins yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(313)
plt.plot(data1['ins']['roll'], 'r', label = 'ins roll')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.show()



# lat, lon std
fig = plt.figure(7)
fig.suptitle('Lat/Lon Standard Deviation', fontsize=30)
plt.subplot(211)
plt.plot(data1['ins']['lat_std'], 'r', label = 'inner imu lat std')
plt.plot(data1['ins']['lat_std'], 'b', label = 'outer imu lat std')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.subplot(212)
plt.plot(data1['ins']['lon_std'], 'r', label = 'inner imu lon std')
plt.plot(data1['ins']['lon_std'], 'b', label = 'outer imu lon std')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.show()

# inspvax position status
fig = plt.figure(8)
fig.suptitle('Position Status', fontsize=30)
plt.subplot(311)
plt.plot(data1['ins']['status'], 'r', label = 'inspvax position status')
plt.plot(data1['bestpos']['status'], 'b', label = 'bestpos position status')
plt.xlabel('stamp')
plt.legend(loc='upper left')
plt.subplot(312)
plt.plot(data1['bestpos']['diff_age'], 'r', label='diff age')
plt.plot(data1['bestpos']['sol_age'], 'b', label='sol age')
plt.legend(loc='upper left')
plt.subplot(313)
plt.plot(data1['ins']['lat_std'], 'r', label = 'inner imu lat std')
plt.legend(loc='upper left')
plt.show()

fig = plt.figure(9)
fig.suptitle('Pitch & Roll', fontsize=30)
plt.subplot(211)
diff = np.mean(data1['ins']['pitch'] - data1['ins']['pitch'])

plt.plot(data1['ins']['pitch'], 'r', label = 'inner imu pitch')
plt.plot(data1['ins']['pitch'] + diff, 'b', label = 'outer imu pitch')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(212)
diff = np.mean(data1['ins']['roll'] - data1['ins']['roll'])
plt.plot(data1['ins']['roll'], 'r', label = 'inner imu roll')
plt.plot(data1['ins']['roll'] + diff, 'b', label = 'outer imu roll')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.show()


