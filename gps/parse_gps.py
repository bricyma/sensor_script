import rosbag
import sys

bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)
filename = '../../dataset/novatel_gps_' + sys.argv[1][:-4] + '.txt'
filename2 = '../../dataset/ublox_gps_' + sys.argv[1][:-4] + '.txt'
gps_file = open(filename, 'w')
gps_file2 = open(filename2, 'w')
for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    lat = msg.latitude
    lng = msg.longitude
    gps_file.write('%f,%f,%f\n' % (lng, lat, 0))

for topic, msg, t in bag.read_messages(topics=['/gps/fix']):
    lat = msg.latitude
    lng = msg.longitude
    gps_file2.write('%f,%f,%f\n' % (lng, lat, 0))

gps_file.close()
gps_file2.close()
bag.close()


