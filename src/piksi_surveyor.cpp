#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
#pragma GCC diagnostic pop

#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <fstream>
#include <iomanip>


size_t counter_gps = 0, counter_gpsrtk = 0;
long double lat_acm = 0.0, lon_acm = 0.0, alt_acm = 0.0, east_acm = 0.0, north_acm = 0.0, up_acm = 0.0;

std::ofstream gps_log, gpsrtk_log;


void gps_callback(const sensor_msgs::NavSatFix  msg)
{
	//Open file at initialization
	if (counter_gps == 0){
		gps_log.open("gps_log.txt");
		gps_log << "Count, Seconds since epoch, NanoSeconds since second, Longitude, Latitude, Altitude, Status;\n";
	}

	//Accumulators
	lat_acm = lat_acm + msg.latitude;
	lon_acm = lon_acm + msg.longitude;
	alt_acm = alt_acm + msg.altitude;

	//Incoming Sample Counter
	counter_gps++;

	//Running Average
	double lat_avg = lat_acm/counter_gps;
	double lon_avg = lon_acm/counter_gps;
	double alt_avg = alt_acm/counter_gps;

	//Write to file
	gps_log << counter_gps << "," << msg.header.stamp.sec << "," << msg.header.stamp.nsec << "," << std::fixed << std::setprecision(10) << msg.latitude << "," << std::fixed << std::setprecision(10) << msg.longitude << "," << std::fixed << std::setprecision(10) << msg.altitude << "," << msg.status.status << ";\n";

	//Output to view sample count and averaged location
	ROS_INFO("[%f] lat	[%f] lon	[%f] alt	[%lu] samples", lat_avg, lon_avg, alt_avg, counter_gps);
}

void gpsrtk_callback(const nav_msgs::Odometry msg)
{
	//Open file at initialization
	if (counter_gpsrtk == 0){
		gpsrtk_log.open("gpsrtk_log.txt");
		gpsrtk_log << "Count, Seconds since epoch, NanoSeconds since second, East(m), North(m), Up(m);\n";
	}

	//Accumulators
	east_acm = east_acm + msg.pose.pose.position.x;
	north_acm = north_acm + msg.pose.pose.position.y;
	up_acm = up_acm + msg.pose.pose.position.z;

	//Incoming Sample Counter
	counter_gpsrtk++;

	//Running Average
	double east_avg = east_acm / static_cast<double>(counter_gpsrtk);
	double north_avg = north_acm / static_cast<double>(counter_gpsrtk);
	double up_avg = up_acm / static_cast<double>(counter_gpsrtk);

	//Write to file
	gpsrtk_log << counter_gpsrtk << "," << msg.header.stamp.sec << "," << msg.header.stamp.nsec << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.x << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.y << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.z << ";\n";
	
	//Output to view sample count and averaged rtk location
	ROS_INFO("[%f] East	[%f] North	[%f] Up	[%lu] samples", east_avg, north_avg, up_avg, counter_gpsrtk);
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "piksi_surveyor");

	ros::NodeHandle n;

	ros::Subscriber sub_gps = n.subscribe("gps/fix", 1, gps_callback);
	ros::Subscriber sub_gpsrtk = n.subscribe("gps/rtkfix", 1, gpsrtk_callback);

	ros::spin();

	return 0;
}









