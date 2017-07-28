#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/TimeReference.h>
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


int counter_gps = 0, counter_gpsrtk = 0;
long double lat_acm = 0, lon_acm = 0, alt_acm = 0, east_acm = 0, north_acm = 0, up_acm = 0;
double lat_avg = 0, lon_avg = 0, alt_avg = 0, east_avg = 0, north_avg = 0, up_avg = 0;
int time_gps_sec = 0, time_gpsrtk_sec = 0, time_gps_nsec = 0, time_gpsrtk_nsec = 0, gps_stat = 0;

std::ofstream gps_log, gpsrtk_log;


void gps_callback(const sensor_msgs::NavSatFix  msg)
{
	

	//Open file at initialization
	if (counter_gps == 0){
		gps_log.open("gps_log.txt");
		gps_log << "Count, Seconds since epoch, NanoSeconds since second, Longitude, Latitude, Altitude, Status;\n";
	}
	
	//Time
	time_gps_sec = msg.header.stamp.sec;
	time_gps_nsec = msg.header.stamp.nsec;

	//Accumalators
	lat_acm = lat_acm + msg.latitude;
	lon_acm = lon_acm + msg.longitude;
	alt_acm = alt_acm + msg.altitude;

	//Incoming Sample Counter
	counter_gps++;

	//Running Average
	lat_avg = lat_acm/counter_gps;
	lon_avg = lon_acm/counter_gps;
	alt_avg = alt_acm/counter_gps;

	//GPS Status
	gps_stat = msg.status.status;

	//Write to file
	gps_log << counter_gps << "," << time_gps_sec << "," << time_gps_nsec << "," << std::fixed << std::setprecision(10) << msg.latitude << "," << std::fixed << std::setprecision(10) << msg.longitude << "," << std::fixed << std::setprecision(10) << msg.altitude << "," << gps_stat << ";\n";

	//Output to view sample count and averaged location
	ROS_INFO("[%f] lat	[%f] lon	[%f] alt	[%d] samples", lat_avg, lon_avg, alt_avg, counter_gps);



}

void gpsrtk_callback(const nav_msgs::Odometry msg)
{
	//Open file at initialization
	if (counter_gpsrtk == 0){
		gpsrtk_log.open("gpsrtk_log.txt");
		gpsrtk_log << "Count, Seconds since epoch, NanoSeconds since second, East(m), North(m), Up(m);\n";
	}
	
	//Time
	time_gpsrtk_sec = msg.header.stamp.sec;
	time_gpsrtk_nsec = msg.header.stamp.nsec;

	//Accumalators
	east_acm = east_acm + msg.pose.pose.position.x;
	north_acm = north_acm + msg.pose.pose.position.y;
	up_acm = up_acm + msg.pose.pose.position.z;

	//Incoming Sample Counter
	counter_gpsrtk++;

	//Running Average
	east_avg = east_acm/counter_gpsrtk;
	north_avg = north_acm/counter_gpsrtk;
	up_avg = up_acm/counter_gpsrtk;

	//Write to file
	gpsrtk_log << counter_gpsrtk << "," << time_gpsrtk_sec << "," << time_gpsrtk_nsec << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.x << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.y << "," << std::fixed << std::setprecision(10) << msg.pose.pose.position.z << ";\n";
	
	//Output to view sample count and averaged rtk location
	ROS_INFO("[%f] East	[%f] North	[%f] Up	[%d] samples", east_avg, north_avg, up_avg, counter_gpsrtk);

}

// void gpstime_callback(const sensor_msgs::TimeReference msg)
// {
// 	//ROS_INFO("I heard: x = [%f], y = [%f] ", msg.x, msg.y);
// 	for(int i = 0; i < 23; i++){

// 		tx[i] = msg->x[i];
// 		ty[i] = msg->y[i];
// 		tpoints[i] = msg->Points[i];
// 		served[i] = msg->Served[i];
// 	}
// }


int main(int argc, char **argv)
{

	ros::init(argc, argv, "piksi_surveyor");

	ros::NodeHandle n;

	ros::Subscriber sub_gps = n.subscribe("gps/fix", 1, gps_callback);
	ros::Subscriber sub_gpsrtk = n.subscribe("gps/rtkfix", 1, gpsrtk_callback);
	// ros::Subscriber sub_gpstime = n.subscribe("gps/time", 1, gpstime_callback);
	
	//ros::Publisher pub = n.advertise<governor::assignment>("assignment", 100);
	
	//ros::Publisher pubhex0 = n.advertise<geometry_msgs::Pose>("/hexacopter0/uav_control/waypoint", 100);


	ros::Rate loop_rate(10);

	
	
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------	
	int loopcount = 0;

	while(ros::ok()){
		
		// ROS_INFO("[%d] Mississippi.", loopcount);

		// geometry_msgs::Pose UAV2set;
		// UAV2set.position.x = tx[vassign[2]];
		// UAV2set.position.y = ty[vassign[2]];
		// UAV2set.position.z = 7;
		// UAV2set.orientation.w = 1;
		// pubhex2.publish(UAV2set);
	
		loopcount++;
		ros::spinOnce();   	    
		loop_rate.sleep();

	}


	return 0;
}









