# gps imu datacheck
1. run parse_gps.py to collect data from rosbag <br />
2. run imu_check.py show the result

3. run orientation.py bagname to get the yaw angle offset. 
diff: course of the vehicle - azimuth of INSPVAX
same as the z angle in the vehiclebodyrotation of Novatel

Used in the auto-calibration of Novatel yaw angle offset.

method: the yaw angle is derived from gps RTK postion  
