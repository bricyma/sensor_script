import rosbag
import sys
import matplotlib.pyplot as plt

bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)
altitude = []
for topic, msg, t in bag.read_messages(topics=['/novatel_data/inspvax']):
    altitude.append(msg.latitude)

bag.close()

plt.ylim([0,1000])

plt.plot(altitude, 'b')
plt.title("Position Type")
plt.show()
