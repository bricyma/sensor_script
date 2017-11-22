#!/bin/bash

echo "STARTING TO ANALYSIS..."
# import rosbag
# . ~/.bashrc
# sudo chmod 755 $(pwd)/$(dirname "$0")/stats/stat_plot.py
# sudo chmod 755 $(pwd)/$(dirname "$0")/test.bag

#sudo chmod 777 util/map.sh

echo ">>> Please enter the number of which you are testing...(1.GPS 2.IMU)"
read choice
if [ 1 -eq $choice ]; then
	echo "This is GPS test."
	#sleep 1s
	echo ">>> Please enter the topic_name...(e.g. /gps/fix)"
	read topic_name
	#gnome-terminal "term2" -x bash -c "sh ./util/map.sh ${topic_name};exec bash;"
	gnome-terminal "term2" -x bash -c "python util/vsimple/server/server.py map;exec bash;"

	STATS_FILE="util/vsimple/plot/BagToMap.py"
	BAG_FILE="testbag/test.bag"
	# echo $STATS_FILE

	python $STATS_FILE $BAG_FILE ${topic_name}

elif [ 2 -eq $choice ]; then
	echo "This is IMU test."
	#sleep 1s
	# echo ">>> Please enter the topic_name...(e.g. /gps/fix)"
	# read topic_name
	# gnome-terminal "term2" -x bash -c "sh ./util/map.sh ${topic_name};exec bash;"
	# gnome-terminal "term2" -x bash -c "python util/vsimple/server/server.py map;exec bash;"
    echo "Currently the IMU test is hard-coded for Xsens, this is due to the lack of parser in Xsens driver, waiting for Driver Pan's further update in this module..."

	STATS_FILE="stats/XsensCom.py"
	BAG_FILE="testbag/test.bag"
	# echo $STATS_FILE

	python $STATS_FILE $BAG_FILE

else echo "Bad choice...Prepare for boom..."

fi