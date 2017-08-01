#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#include <geometry_msgs/PointStamped.h>
#pragma GCC diagnostic pop

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>


size_t counter_gps = 0, counter_gpsrtk = 0;
long double lat_acm = 0.0, lon_acm = 0.0, alt_acm = 0.0, east_acm = 0.0, north_acm = 0.0, up_acm = 0.0;

boost::filesystem::ofstream gps_log, gpsrtk_log;

void point_callback(const geometry_msgs::PointStamped& msg)
{
	//Accumulators
	east_acm = east_acm + msg.point.x;
	north_acm = north_acm + msg.point.y;
	up_acm = up_acm + msg.point.z;

	//Incoming Sample Counter
	counter_gps++;

	long double east_avg = east_acm / static_cast<long double>(counter_gps);
	long double north_avg = north_acm / static_cast<long double>(counter_gps);
	long double up_avg = up_acm / static_cast<long double>(counter_gps);

	//Write to file
	gps_log << counter_gps << "," << msg.header.stamp.sec << "," << msg.header.stamp.nsec << "," << std::fixed << std::setprecision(10) << msg.point.x << "," << std::fixed << std::setprecision(10) << msg.point.y << "," << std::fixed << std::setprecision(10) << msg.point.z << ";\n";

	//Output to view sample count and averaged rtk location
	ROS_INFO_THROTTLE(2,"[%.6Lf] X [%.6Lf] Y [%.6Lf] Z [%lu] samples", east_avg, north_avg, up_avg, counter_gps);
}


void gps_callback(const sensor_msgs::NavSatFix  msg)
{
	//Accumulators
	lat_acm += msg.latitude;
	lon_acm += msg.longitude;
	alt_acm += msg.altitude;

	//Incoming Sample Counter
	counter_gps++;

	//Running Average
	long double lat_avg = lat_acm / static_cast<long double>(counter_gps);
	long double lon_avg = lon_acm / static_cast<long double>(counter_gps);
	long double alt_avg = alt_acm / static_cast<long double>(counter_gps);

	//Write to file
	gps_log << counter_gps << "," << msg.header.stamp.sec << "," << msg.header.stamp.nsec << "," << std::fixed << std::setprecision(10) << msg.latitude << "," << std::fixed << std::setprecision(10) << msg.longitude << "," << std::fixed << std::setprecision(10) << msg.altitude << "," << msg.status.status << ";\n";

	//Output to view sample count and averaged location
	ROS_INFO_THROTTLE(2,"[%.8Lf] lat [%.8Lf] lon [%.6Lf] alt [%lu] samples", lat_avg, lon_avg, alt_avg, counter_gps);
}

void gpsrtk_callback(const nav_msgs::Odometry msg)
{
	//Accumulators
	east_acm += msg.pose.pose.position.x;
	north_acm += msg.pose.pose.position.y;
	up_acm += msg.pose.pose.position.z;

	//Incoming Sample Counter
	counter_gpsrtk++;

	//Running Average
	long double east_avg = east_acm / static_cast<long double>(counter_gpsrtk);
	long double north_avg = north_acm / static_cast<long double>(counter_gpsrtk);
	long double up_avg = up_acm / static_cast<long double>(counter_gpsrtk);

	//Write to file
	gpsrtk_log << counter_gpsrtk << "," << msg.header.stamp.sec << "," << msg.header.stamp.nsec << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.x << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.y << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.z << ";\n";
	
	//Output to view sample count and averaged rtk location
	ROS_INFO_THROTTLE(2,"[%.6Lf] East [%.6Lf] North [%.6Lf] Up [%lu] samples", east_avg, north_avg, up_avg, counter_gpsrtk);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "piksi_surveyor");

	ros::NodeHandle n;

	ros::NodeHandle pnh("~");

	bool gps_mode = true;

	pnh.getParam("gps_mode", gps_mode);

	std::string file_name("gps");
	pnh.getParam("base_file_name", file_name);

	std::string logging_dir("~");
	if (pnh.getParam("logging_dir", logging_dir))
	{
		ROS_INFO("logging to %s", logging_dir.c_str());
	}

	if (gps_mode)
	{
		std::string gps_suffix("_log.csv");
		std::string rtk_suffix("_rtk_log.csv");

		std::string gps_path(logging_dir);
		gps_path.append("/");
		gps_path.append(file_name);
		gps_path.append(gps_suffix);

		std::string rtk_path(logging_dir);
		rtk_path.append("/");
		rtk_path.append(file_name);
		rtk_path.append(rtk_suffix);

		ROS_INFO("Attempting to log GPS data to %s and %s", gps_path.c_str(), rtk_path.c_str());

		boost::filesystem::path boost_gps_path(gps_path);
		boost::filesystem::path boost_rtk_path(rtk_path);

		if (boost::filesystem::exists(boost_gps_path))
		{
			ROS_ERROR("%s exists!", gps_path.c_str());
			return -1;
		}

		if (boost::filesystem::exists(boost_rtk_path))
		{
			ROS_ERROR("%s exists!", rtk_path.c_str());
			return -1;
		}

		gps_log.open(boost_gps_path);
		gpsrtk_log.open(boost_rtk_path);

		gps_log << "Count, Seconds since epoch, NanoSeconds since second, Longitude, Latitude, Altitude, Status;\n";
		gpsrtk_log << "Count, Seconds since epoch, NanoSeconds since second, East(m), North(m), Up(m);\n";

		// subscribing to gps data
		ros::Subscriber sub_gps = n.subscribe("gps/fix", 1, gps_callback);
		ros::Subscriber sub_gpsrtk = n.subscribe("gps/rtkfix", 1, gpsrtk_callback);

		ros::spin();

		gps_log.close();
		gpsrtk_log.close();
	}
	else
	{
		std::string point_suffix("_log.csv");

		std::string point_path(logging_dir);
		point_path.append("/");
		point_path.append(file_name);
		point_path.append(point_suffix);

		ROS_INFO("Attempting to log Point data to %s", point_path.c_str());

		boost::filesystem::path boost_point_path(point_path);

		if (boost::filesystem::exists(boost_point_path))
		{
			ROS_ERROR("%s exists!", point_path.c_str());
			return -1;
		}

		gps_log.open(boost_point_path);

		gps_log << "Count, Seconds since epoch, NanoSeconds since second, X (m), Y (m), Z (m);\n";

		ros::Subscriber sub = n.subscribe("point", 1, point_callback);

		ros::spin();

		gps_log.close();
	}


	

	return 0;
}









