import rosbag

bagname = '2017-04-03-11-35-04.bag'
bagname = '../rosbag/' + bagname
bag = rosbag.Bag(bagname)
gps_file = open('gps_data.txt', 'w')
sequence = 0
for topic, msg, t in bag.read_messages(topics=['/novatel_data/bestpos']):
    #s = msg.linear
    time1 = msg.header.gps_week_seconds
    # sequence = msg.header2.seq
    # time1 = msg.header2.stamp.nsecs
    # time1 = msg.header2.stamp.to_sec()
    print msg.header2.stamp, msg.header.gps_week_seconds
    # time1 = msg.header2.stamp
    # print msg.header2.stamp.secs, msg.header2.stamp.nsecs, msg.header2.stamp.to_sec(), msg.header.gps_week_seconds
    sequence += 1
    time2 = 0
    # time1 = msg.header.gps_week_seconds
    s =  str(sequence) + ' ' + str(time1) + ' ' +  str(time2)
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
    diff_x.append(count)
    # print float(l[1]) - before
    # if int(l[1]) - before < 0:
    if float(l[1])-before != 50 and count > 0:
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
