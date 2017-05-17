import rosbag
import sys

bagname = '../rosbag/'
bagname += sys.argv[1]

bag = rosbag.Bag(bagname)
gps_file = open('gps_data.txt', 'w')
sequence = 0
for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    yaw = msg.azimuth
    if yaw < 0:
        print orientation
    gps_file.write('%f \n' % yaw)
gps_file.close()
bag.close()


