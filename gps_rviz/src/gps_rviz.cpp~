#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"
#include "gps_handler.cpp"



void gps2rviz(const sensor_msgs::NavSatFix gps_NavSatFix){ //callback function

	geometry_msgs::Vector3 gpsOut;

	gpsOut.x = gps_NavSatFix.latitude; 
	gpsOut.y = gps_NavSatFix.longitude;
	gpsOut.z = gps_NavSatFix.altitude;

	ROS_INFO("I extracted: lat=[%f] , lon=[%f] and alt=[%f]" ,gpsOut.x, gpsOut.y, gpsOut.z);
	
	geometry_msgs::Vector3 cleaned_GPS_point;
	cleaned_GPS_point = gps::gps2cart(gpsOut);	

	gps::addToTrack(cleaned_GPS_point);	
	
	gps::publishGPS_Track();
} 


int main(int argc, char **argv)
{	
	ROS_INFO("You are running playground!!!");
	ros::init(argc, argv, "nGPSlistener");
	
	ros::NodeHandle n;

	gps::initializeTrack();
	
	ros::Subscriber sub = n.subscribe("/gps/fix" , 1000, gps2rviz);

	ros::spin();
	return 0;
}
