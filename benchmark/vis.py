# compare imu result in different positions 
from parse import DataParser
import matplotlib.pyplot as plt
import numpy as np
import sys
from gnss_imu_benchmark import IMUAnalyzer
 
# B2 Standard
bag_name = '2018-05-17-10-50-21'
# bag_name = '2018-05-16-14-38-07'

bag_name = '2018-05-16-10-54-03'
# bag_name = '2018-05-07-16-05-02'
# bag_name = '2018-05-08-14-48-21'
# bag_name = '2018-05-10-17-16-22'

# human driving
# bag_name = '2018-05-21-11-17-12'
# bag_name = '2018-05-21-16-15-06'
# bag_name = '2018-05-21-17-40-08'
# B1
# bag_name = '2018-05-18-11-29-26'
# bag_name = '2018-05-21-10-49-42'
# bag_name = '2018-05-22-14-22-23'
# bag_name = '2018-05-23-10-52-25'
ts_begin = '0:00'
ts_end = '100:01'
info = {'bag_name': bag_name, 'ts_begin': ts_begin, 'ts_end': ts_end}
analyzer = IMUAnalyzer(info, 1)

data1 = analyzer.data1
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
plt.plot(data1['corrimu']['x_acc'], 'r', linewidth =1,  label = 'acc_x in imu')
plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.subplot(312)
plt.plot(data1['corrimu']['y_acc'], 'r', linewidth =1, label = 'acc_y in imu')
plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
plt.subplot(313)
plt.plot(data1['corrimu']['z_acc'], 'r', linewidth =1, label = 'acc_z in imu')
plt.xlabel('stamp')
plt.ylabel('m/s2')
plt.legend(loc='upper left')
    
# angular rate
fig = plt.figure(2)
fig.suptitle('Angular rate in 3 axle', fontsize=30)
plt.subplot(411)
plt.plot(data1['corrimu']['pitch_rate'], 'r', linewidth =1,  label = 'picth_rate in imu')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('rad/s')
plt.subplot(412)
plt.plot(data1['corrimu']['roll_rate'], 'r', linewidth =1, label = 'roll_rate in imu')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('rad/s')
plt.subplot(413)
plt.plot(data1['corrimu']['yaw_rate'], 'r', linewidth =1, label = 'yaw_rate in imu')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('rad/s')
plt.subplot(414)
diff1 = data1['corrimu']['yaw_rate'][1:] - data1['corrimu']['yaw_rate'][:-1]
print 'std of yaw rate diff: ',  format(np.std(diff1), '.4f')
plt.plot(diff1, 'r', linewidth =1, label = 'yaw_rate change in imu')
plt.xlabel('stamp')
plt.ylabel('rad/s/stamp')
plt.legend(loc='upper left')


# timestamp, packet loss
fig = plt.figure(3)
fig.suptitle('Timestamp space', fontsize = 30)
diff1 = data1['ins']['t'][1:] - data1['ins']['t'][:-1]
diff2 = data1['ins']['t_gps'][1:] - data1['ins']['t_gps'][:-1]
print 'std of ros timestamp: ', format(np.std(diff1), '.4f')
plt.plot(diff1, 'r', label = 'ros timestamp')
plt.plot(diff2, 'b', label = 'gps timestamp')
plt.xlabel('stamp')
plt.ylabel('time/s')
plt.legend(loc='upper left')

# calibration result
fig = plt.figure(4)
fig.suptitle('Yaw angle calibration result', fontsize=30)
plt.subplot(211)
plt.plot(data1['ins']['yaw'], 'r', label = 'ins yaw')
plt.plot(data1['spd']['yaw'], 'b', label = 'spd yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(212)
diff = data1['ins']['yaw'] - data1['spd']['yaw']
diff[abs(diff) > 0.5] = 0
print 'mean of (ins yaw - spd yaw): ', format(np.mean(diff), '.6f')
plt.plot(diff, 'r', label = 'inspvax - insspd')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')


fig = plt.figure(5)
fig.suptitle('Vehicle position compared with lane center', fontsize=30)
plt.subplot(211)
plt.plot(data1['pos']['x'], 'r', label = 'pos offset x between inspvax and lane center')
plt.plot(data1['pos']['y'], 'b', label = 'pos offset y between inspvax and lane center')
plt.legend(loc='upper left')

# leverarm result
#  -1.037, 2.78
# outer: -0.89, 3.4

# actual: 1.038, 2.76
# 0.838, 3.31
fig = plt.figure(6)
fig.suptitle('Leverarm', fontsize=30)
plt.subplot(411)
plt.plot(data1['offset']['x'], 'r', label = 'leverarm offset x')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.subplot(412)
plt.plot(data1['offset']['y'], 'r', label = 'leverarm offset y')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.subplot(413)
plt.plot(data1['ins']['yaw'], 'r', label = 'ins yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(414)
plt.plot(data1['ins']['roll'], 'r', label = 'ins roll')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')



# lat, lon std
fig = plt.figure(7)
fig.suptitle('Lat/Lon Standard Deviation', fontsize=30)
plt.subplot(211)
plt.plot(data1['ins']['lat_std'], 'r', label = 'imu lat std')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')
plt.subplot(212)
plt.plot(data1['ins']['lon_std'], 'r', label = 'imu lon std')
plt.xlabel('stamp')
plt.ylabel('m')
plt.legend(loc='upper left')

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
plt.plot(data1['ins']['lat_std'], 'r', label = 'imu lat std')
plt.legend(loc='upper left')

fig = plt.figure(9)
fig.suptitle('Pitch & Roll', fontsize=30)
plt.subplot(211)
plt.plot(data1['ins']['pitch'], 'r', label = 'imu pitch')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(212)
plt.plot(data1['ins']['roll'], 'r', label = 'imu roll')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')


fig = plt.figure(10)
fig.suptitle('INS yaw vs SPD yaw vs HDmap yaw')
plt.subplot(211)
plt.plot(data1['ins']['yaw'], 'r', label = 'ins  yaw')
plt.plot(data1['spd']['yaw'], 'b', label = 'spd  yaw')
plt.plot(data1['map']['yaw'], 'g', label = 'hdmap  yaw')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')
plt.subplot(212)
diff_ins = data1['ins']['yaw'] - data1['map']['yaw']
diff_spd = data1['spd']['yaw'] - data1['map']['yaw']
diff_ins[abs(diff_ins)>1] = 0
diff_spd[abs(diff_spd)>1] = 0
plt.plot(diff_ins, 'r', label = 'ins - map')
plt.plot(diff_spd, 'b', label = 'spd - map')
plt.xlabel('stamp')
plt.ylabel('deg')
plt.legend(loc='upper left')

print 'average of (ins -map): ', format(np.mean(diff_ins), '.4f')
print 'average of (spd -map): ', format(np.mean(diff_spd), '.4f')


fig = plt.figure(11)
fig.suptitle('INS timestamp vs SPD timestamp')
plt.subplot(211)
plt.plot(data1['ins']['t'], 'r', label = 'ins timestamp')
plt.plot(data1['spd']['t'], 'b', label = 'spd timestamp')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('t/s')
plt.subplot(212)
diff = data1['ins']['t'] - data1['ins']['t_gps']
diff2 = data1['ins']['t_gps'] - data1['spd']['t_gps']
plt.plot(diff, 'r', label = 'ins t - spd t')
plt.plot(diff2, 'b', label = 'ins t_gps - spd t_gps')
plt.legend(loc='upper left')
plt.xlabel('stamp')
plt.ylabel('t/s')
plt.show()
