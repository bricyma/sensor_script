import rosbag
import matplotlib.pyplot as plt

def plot(x, y):
    plt.ylim([0,100])
    plt.plot(x, y)
    # plt.plot([1,2,3,4], [1,4,9,16], 'ro')
    # plt.axis([0, 6, 0, 20])
    plt.show()
    print 'plot finish'


diff_x = []
diff_y = []

bagname = '2017-03-24-10-51-56.bag'
bagname = '../rosbag/2017-3-24/' + bagname
# bagname = '/mnt/scratch/datasets/2017-03-20-1/' + bagname
bag = rosbag.Bag(bagname)
gps_file = open('gps_data.txt', 'w')
sequence = 0
for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    #s = msg.linear
    # sequence = msg.header2.seq
    # time1 = msg.header2.stamp.nsecs
    # time2 = msg.header2.stamp.to_sec()
    sequence += 1
    time2 = 0
    time1 = msg.header.gps_week_seconds
    s =  str(sequence) + ' ' + str(time1) + ' ' + str(time2)
    # print s
    gps_file.write('%s \n' % s)
gps_file.close()
bag.close()


file_path = 'gps_data.txt'
input_file = open(file_path, 'r')
before = 0
loss_num = 0
count = 0
for line in input_file:
    l = line.split(' ')
    diff_y.append(int(l[1])-before)
    print int(l[1])-before
    diff_x.append(count)
    if int(l[1])-before != 20:
        print l[1], loss_num
        loss_num += 1
    before = int(l[1])
    count += 1

data_file = open('packet_loss.txt', 'w')
data_file.write("bagname: %s\n" % bagname)
data_file.write("total sequences: %d\n" % sequence)
data_file.write("packet loss: %d\n" % loss_num)
data_file.write("loss rate: %s\n" % str(float(loss_num)/sequence))

data_file.close()

print bagname
print "total seq: ", sequence
print "loss packet: ", loss_num
print "loss rate: ", float(loss_num)/sequence
	
plot(diff_x, diff_y)