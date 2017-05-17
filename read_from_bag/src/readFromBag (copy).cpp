#include <string>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_datatypes.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <read_from_bag/Ack.h>
#include <read_from_bag/BESTPOS.h>
#include <read_from_bag/CORRIMUDATA.h>
#include <read_from_bag/INSCOV.h>
#include <read_from_bag/INSPVAX.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

using std::cout;
using std::endl;
using std::string;

const static double pi = 3.1415926;
const double LiDAR_CAM_DELAY = 0;

int main()
{
	int mode = 4; // 1 for GPS/IMU, 2 for velodyne, 3 for left image, 4 for right image, 5 for INSPVAX

    rosbag::Bag bag;
    // bag.open("/mnt/scratch/yiluo/US_demo_car/raw/2017-01-25-20-05-24.bag", rosbag::bagmode::Read);
    bag.open("/home/yiluo/2017-03-10-18-08-59.bag", rosbag::bagmode::Read);
    // bag.open("/home/yiluo/Fusion/data/0220/2017-02-20-19-45-08.bag", rosbag::bagmode::Read);
    
    std::vector<std::string> topics;
    if(mode == 1)
    {
	    topics.push_back(std::string("/imu/data"));
	    topics.push_back(std::string("/navsat/fix"));
	    // topics.push_back(std::string("/vehicle/gps/fix"));
	}
	else if(mode == 2)
	{
    	topics.push_back(std::string("/velodyne_points"));
	}
	else if(mode == 3)
	{
		topics.push_back(std::string("/camera2/image_color/compressed"));
	}
    else if(mode == 4)
    {   
        topics.push_back(std::string("/camera3/image_color/compressed"));
    }
    else if(mode == 5)
    {
        topics.push_back(std::string("/novatel_data/inspvax"));   
    }

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    
    std::ofstream IMUGPSFileOut;
    std::ofstream RAWIMUGPSFileOut;
    std::ofstream PointcloudFileOut;
    std::ofstream ImageFileOut;

    string MainFolder = "20170310/cali";
    string IMUGPSFolderPath = "/mnt/scratch/yiluo/car_record/map/" + MainFolder + "/imugps_dat/";
    string IMUGPSFilePath = "/mnt/scratch/yiluo/car_record/map/" + MainFolder + "/imugps_dat/imugps_record.txt";
    string RAWIMUGPSFilePath = "/mnt/scratch/yiluo/car_record/map/" + MainFolder + "/imugps_dat/rawimugps_record.txt";
    string PointcloudRecordFilePath = "/mnt/scratch/yiluo/car_record/map/" + MainFolder +"/pointcloud_record.txt";
    string pointcloudCSVFolderPath = "/mnt/scratch/yiluo/car_record/map/" + MainFolder + "/lidar_dat/";
    // Use left camera by default
    string ImageRecordFilePath = "/mnt/scratch/yiluo/car_record/map/" + MainFolder + "/img2_record.txt";
    string ImageFileFolderPath = "/mnt/scratch/yiluo/car_record/map/"+ MainFolder +"/img2_dat/";
    if(mode == 4)
    {
        ImageRecordFilePath = "/mnt/scratch/yiluo/car_record/map/" + MainFolder + "/img3_record.txt";
        ImageFileFolderPath = "/mnt/scratch/yiluo/car_record/map/"+ MainFolder +"/img3_dat/";
    }

    string command;

    if(mode == 1)
    {
        command = "mkdir -p " + IMUGPSFolderPath;
        system(command.c_str());
        IMUGPSFileOut.open(IMUGPSFilePath.c_str());
        if(!IMUGPSFileOut.is_open())
        {
            cout << "Could not open file!\n";
            return -1;
        }
        
    }
    else if(mode == 2)
    {   
        command = "mkdir -p " + pointcloudCSVFolderPath;
        system(command.c_str());
        PointcloudFileOut.open(PointcloudRecordFilePath.c_str());
        if(!PointcloudFileOut.is_open())
        {
            cout << "Could not open file!\n";
            return -1;
        }
    }
    else if(mode == 3 || mode == 4)
    {	
        command = "mkdir -p " + ImageFileFolderPath;
        system(command.c_str());    
    	ImageFileOut.open(ImageRecordFilePath.c_str());	
        if(!ImageFileOut.is_open())
        {
            cout << "Could not open file!\n";
            return -1;
        }
    }
    else if(mode == 5)
    {
        command = "mkdir -p " + IMUGPSFolderPath;
        system(command.c_str());
        RAWIMUGPSFileOut.open(RAWIMUGPSFilePath.c_str());
        if(!RAWIMUGPSFileOut.is_open())
        {
            cout << "Could not open file!\n";
            return -1;
        }
    }

    int counter = 0;
    bool gpsIsSet = false;
    double server_time, ay, ax, az, altitude, gps_yaw, wz, latitude, wx, hz, hx, hy, angley, anglex, wy, anglez, longitude;

    // allocate space first
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;

    foreach(rosbag::MessageInstance const m, view)
    {
        
        if(mode == 1)
        {
        	sensor_msgs::Imu::ConstPtr imu_data = m.instantiate<sensor_msgs::Imu>();
        	if (imu_data != NULL)
        	{
            	cout << "IMU: " << imu_data->header.stamp << ", ";
            	std::cout << imu_data->orientation.x << " ";
            	std::cout << imu_data->angular_velocity.x << " ";
            	std::cout << imu_data->linear_acceleration.x << "\n";

            	server_time = imu_data->header.stamp.toSec();
            	anglex = imu_data->orientation.x;
            	angley = imu_data->orientation.y;
            	anglez = imu_data->orientation.z;
            	// gps_yaw = imu_data->orientation.w;
            	gps_yaw = 0.0;
            	// hx = imu_data->orientation.w;
            	hx = 0.0;
            	hy = 0.0;
            	hz = 0.0;
            	ax = imu_data->linear_acceleration.x;
            	ay = imu_data->linear_acceleration.y;
            	az = imu_data->linear_acceleration.z;
            	wx = imu_data->angular_velocity.x;
            	wy = imu_data->angular_velocity.y;
            	wz = imu_data->angular_velocity.z;

                // altitude = 0.0;
                // latitude = 0.0;
                // longitude = 0.0;
                // gpsIsSet = true;

            	if(gpsIsSet)
            	{
         			char stemp[1000];
         			// server_time, ay, ax, az, altitude, wz, latitude, wx, hz, hx, hy, angley, anglex, wy, anglez, longitude
	                // sprintf(stemp, "{\"server_time\":\"%6.20f\",\"gps_height\":0,\"vdop\":0,\"ay\":%6.20f,\"ax\":%6.20f,\"az\":%6.20f,\"temperature\":0,\"altitude\":%6.20f,\"gps_yaw\":0,\"wz\":%6.20f,\"latitude\":%6.20f,\"wx\":%6.20f,\"sn\":0,\"hz\":%6.20f,\"hx\":%6.20f,\"hy\":%6.20f,\"hdop\":0,\"pressure\":0,\"angley\":%6.20f,\"anglex\":%6.20f,\"wy\":%6.20f,\"anglez\":%6.20f,\"pdop\":0,\"longitude\":%6.20f,\"ground_velocity\":0,\"local_time\":\"2016-11-24 13:36:36.1011850\",\"time\":\"2016-11-24 5:36:36.101185\"}",
	                   // server_time, ay, ax, az, altitude, wz, latitude, wx, hz, hx, hy, angley, anglex, wy, anglez, longitude);	
	                sprintf(stemp, "{\"server_time\":\"%6.20f\",\"gps_height\":0,\"vdop\":0,\"ay\":%6.20f,\"ax\":%6.20f,\"az\":%6.20f,\"temperature\":0,\"altitude\":%6.20f,\"gps_yaw\":%6.20f,\"wz\":%6.20f,\"latitude\":%6.20f,\"wx\":%6.20f,\"sn\":0,\"hz\":%6.20f,\"hx\":%6.20f,\"hy\":%6.20f,\"hdop\":0,\"pressure\":0,\"angley\":%6.20f,\"anglex\":%6.20f,\"wy\":%6.20f,\"anglez\":%6.20f,\"pdop\":0,\"longitude\":%6.20f,\"ground_velocity\":0,\"local_time\":\"2016-11-24 13:36:36.1011850\",\"time\":\"2016-11-24 5:36:36.101185\"}",
	                   server_time, ay, ax, az, altitude, gps_yaw, wz, latitude, wx, hz, hx, hy, angley, anglex, wy, anglez, longitude);   
                    // sprintf(stemp, "%6.10f\t%6.10f\t%6.10f\t%6.10f\t%6.10f\t%6.10f\t%6.10f\t%6.10f\t%6.10f\t%6.10f\t%6.10f",
                    //    server_time, ax/50.0, ay/50.0, az/50.0, wx/50.0, wy/50.0, wz/50.0, anglex, angley, anglez, hx);   
	                IMUGPSFileOut << stemp << endl;
            	}

        	}
        	else
        	{
	            sensor_msgs::NavSatFix::ConstPtr gps_data = m.instantiate<sensor_msgs::NavSatFix>();
	            if (gps_data != NULL)
	            {
	                cout << "GPS: " << gps_data->header.stamp << ", ";
	                cout << std::setprecision(12) << gps_data->latitude << " ";
	                cout << std::setprecision(12) << gps_data->longitude << " ";
	                cout << std::setprecision(12) << gps_data->altitude << "\n";

	                // char stemp[1000];
	                // // server_time altitude latitude longitude
	                // sprintf(stemp, "{\"server_time\":\"%6.20f\",\"gps_height\":0,\"vdop\":0,\"ay\":0,\"ax\":0,\"az\":0,\"temperature\":0,\"altitude\":%6.20f,\"gps_yaw\":0,\"wz\":0,\"latitude\":%6.20f,\"wx\":0,\"sn\":0,\"hz\":0,\"hx\":0,\"hy\":0,\"hdop\":0,\"pressure\":0,\"angley\":0,\"anglex\":0,\"wy\":0,\"anglez\":0,\"pdop\":0,\"longitude\":%6.20f,\"ground_velocity\":0,\"local_time\":\"2016-11-24 13:36:36.1011850\",\"time\":\"2016-11-24 5:36:36.101185\"}",
	                //    gps_data->header.stamp.toSec(), gps_data->altitude, gps_data->latitude, gps_data->longitude);
	                // // sprintf(stemp, "%6.20f %6.20f %6.20f", gps_data->altitude, gps_data->latitude, gps_data->longitude);
	                // IMUGPSFileOut << stemp << endl;

	                if(!gpsIsSet)
	                {
	                	gpsIsSet = true;
	                }
	                altitude = gps_data->altitude;
	                latitude = gps_data->latitude;
	                longitude = gps_data->longitude;
	            }	
        	}
        }
        else if(mode == 2)
        {
            sensor_msgs::PointCloud2::ConstPtr pointcloud_data = m.instantiate<sensor_msgs::PointCloud2>();
            char stemp[1000];
            std::ofstream csvFile;
            laserCloudIn.clear();
            
            string numFileName;
            if (pointcloud_data != NULL)
            {
                double timeLaserOdometry = pointcloud_data->header.stamp.toSec() - LiDAR_CAM_DELAY;

                numFileName.clear();
                std::stringstream StrStm;
                StrStm << std::setprecision(12) << timeLaserOdometry;
                StrStm >> numFileName;
                
                pcl::fromROSMsg(*pointcloud_data, laserCloudIn);
                if((counter%1000) == 0)
                {
                    cout << "Pointcloud: " << pointcloud_data->header.stamp << ", ";
                    cout << laserCloudIn.points.size() << "\n";
                }

                string pointcloudCSVFileName = pointcloudCSVFolderPath + numFileName + ".csv";
                
                csvFile.open(pointcloudCSVFileName.c_str());
                for (int i = 0; i < laserCloudIn.points.size(); ++i)
                {
                    csvFile << std::setprecision(12) << laserCloudIn.points[i].x << ",";
                    csvFile << std::setprecision(12) << laserCloudIn.points[i].y << ",";
                    csvFile << std::setprecision(12) << laserCloudIn.points[i].z << ",";
                    csvFile << std::setprecision(12) << timeLaserOdometry << ",";
                    csvFile << "\n";
                }
                csvFile.close();

                // timestamp path                
                sprintf(stemp, "%6.20f\t%s", timeLaserOdometry, pointcloudCSVFileName.c_str());
                PointcloudFileOut << stemp << endl;

                counter++;
            }
        }
        else if(mode == 3 || mode == 4)
        {
        	sensor_msgs::CompressedImage::ConstPtr image_data = m.instantiate<sensor_msgs::CompressedImage>();
        	cout << "IMG: " << image_data->header.stamp << "\n";
        	// cout << image_data->format << "\n";

        	double timeLaserOdometry = image_data->header.stamp.toSec();
            string numFileName;
            std::stringstream StrStm;
            StrStm << std::setprecision(12) << timeLaserOdometry;
            StrStm >> numFileName;
        	
        	string ImageFileName = ImageFileFolderPath + numFileName + ".jpeg";

        	cv_bridge::CvImagePtr cv_ptr;
		    try
		    {
		      cv_ptr = cv_bridge::toCvCopy(image_data, sensor_msgs::image_encodings::BGR8);
		    }
		    catch (cv_bridge::Exception& e)
		    {
		      ROS_ERROR("cv_bridge exception: %s", e.what());
		      return 0;
		    }

		    cv::Mat img = cv_ptr->image;
        	cv::imwrite(ImageFileName.c_str(), img);
        	
        	char stemp[1000];
        	// timestamp path                
            sprintf(stemp, "%6.20f\t%s", timeLaserOdometry, ImageFileName.c_str());
            ImageFileOut << stemp << endl;
        }
        else if (mode == 5)
        {
            read_from_bag::INSPVAX::ConstPtr fusion_data = m.instantiate<read_from_bag::INSPVAX>();
            cout << "INSPVAX: " << fusion_data->header2.stamp.toSec() << "\n";

            anglex = fusion_data->roll;
            angley = fusion_data->pitch;
            anglez = fusion_data->azimuth;

            // anglex = fusion_data->roll/180.0*pi;
            // angley = fusion_data->pitch/180.0*pi;
            // anglez = fusion_data->azimuth/180.0*pi;

            // tf::Quaternion q(angley, anglex, anglez);
            // tf::Matrix3x3 m(q);
            // double roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);
            // std::cout << "Roll: " << anglex << ", Pitch: " << angley << ", Yaw: " << anglez << ", ";
            // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;

            // double timestamp = fusion_data->header2.stamp.toSec();
            double timestamp = fusion_data->header.gps_week_seconds;

            latitude = fusion_data->latitude;
            longitude = fusion_data->longitude;
            altitude = fusion_data->altitude;

            double roll_std = fusion_data->roll_std;
            double pitch_std = fusion_data->pitch_std;
            double azimuth_std = fusion_data->azimuth_std;

            double latitude_std = fusion_data->latitude_std;
            double longitude_std = fusion_data->longitude_std;
            double altitude_std = fusion_data->altitude_std;

            char stemp[1000];
                    
            sprintf(stemp, "%6.20f %6.20f %6.20f %6.20f %6.20f %6.20f %6.20f %6.20f %6.20f %6.20f %6.20f %6.20f %6.20f",
                       timestamp,
                       anglex, angley, anglez, 
                       latitude, longitude, altitude,
                       roll_std, pitch_std, azimuth_std,
                       latitude_std, longitude_std, altitude_std);
                    
            RAWIMUGPSFileOut << stemp << endl;

        }
    }

    if(mode = 1)
    {
        IMUGPSFileOut.close();
    }
    else if(mode == 2)
    {   
        PointcloudFileOut.close();
    }
    else if(mode == 3 || mode == 4)
    {
    	ImageFileOut.close();
    }
    else if(mode == 5)
    {
        RAWIMUGPSFileOut.close();
    }

    bag.close();
    return 0;
}
