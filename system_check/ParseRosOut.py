import rosbag
import math as m
import numpy as np
import matplotlib.pyplot as plt
import sys

bagname = '../../rosbag/'
bagname += sys.argv[1]
bag = rosbag.Bag(bagname)

print bagname

count = dict()
seqcount = dict()
for x in bag.read_messages(topics=['/rosout']):
  total = x[1]
  if total.name[:7] == '/record':
    continue
  header_time = total.header.stamp.to_sec()
  msg = total.msg
  if msg.find('publish time') == -1:
    continue
  name = msg[:msg.find('in ') - 1]
  publisher_time = float(msg[msg.find('time: ') + 6:])
  sequence = int(msg[msg.find('sequence: ') + 10: msg.find(', publish time')])
  if count.has_key(name):
    count[name].append(header_time - publisher_time)
    seqcount[name].append(sequence)
  else:
    count[name] = []
    seqcount[name] = []
    count[name].append(header_time - publisher_time)
    seqcount[name].append(sequence)

diff = dict()
for x in seqcount.keys():
   # if x == "velodyne_packets":
   #   continue
   diff[x] = []
   for i in range(1, len(seqcount[x])):
     diff[x].append(seqcount[x][i] - seqcount[x][i-1] - 1) 
   print x, np.sum(diff[x])
   plt.title("package loss number of " + x)
   plt.plot(diff[x])
   plt.show()


for x in count.keys():
  # if x == "velodyne_packets":
  #   continue
  print x, np.mean(count[x])
  plt.title("time cost of " + x)
  plt.plot(count[x])
  plt.show()


bag.close()

