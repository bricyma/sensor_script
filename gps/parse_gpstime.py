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
    gps_seconds = msg.header.gps_week_seconds
    gps_file.write('%d,%f,%f,%f\n' % (gps_seconds, lng, lat, 0))

for topic, msg, t in bag.read_messages(topics=['/gps/navposllh']):
    lat = msg.lat/10000000.0
    lng = msg.lon/10000000.0
    gps_seconds = msg.iTOW
    gps_file2.write('%d,%f,%f,%f\n' % (gps_seconds, lng, lat, 0))

gps_file.close()
gps_file2.close()
bag.close()


