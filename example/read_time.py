import re
file_path = 'gps_data.txt'
input_file = open(file_path, 'r')

gps_file = open('gps_data_1124.txt', 'w')
before = 0
for line in input_file:
	l = line.split(' ')
	# if int(l[2])-before != 20:
	print l[2]
	before = int(l[1])
	

	
