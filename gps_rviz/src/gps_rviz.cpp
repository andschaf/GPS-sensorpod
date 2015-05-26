#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"
#include "gps_handler.cpp"


void publishGPS_Track(){

	ros::NodeHandle n; //TODO necessary??
	ros::Publisher gps_pub = n.advertise<visualization_msgs::MarkerArray>("GPS_marker",1000); //TODO rewrite this, nicht jedesmal advertisen
	gps_pub.publish(gps::GPS_TRACK);
	ros::spin();
}

void gps_callback(const sensor_msgs::NavSatFix gps_NavSatFix){ //callback function

	geometry_msgs::Vector3 gpsOut;

	gpsOut.x = gps_NavSatFix.latitude; 
	gpsOut.y = gps_NavSatFix.longitude;
	gpsOut.z = gps_NavSatFix.altitude;
	
	geometry_msgs::Vector3 cleaned_GPS_point;
	cleaned_GPS_point = gps::gps2cart(gpsOut);	

	gps::addToTrack(cleaned_GPS_point);	
	
	publishGPS_Track(); 
} 



int main(int argc, char **argv)
{	
	ROS_INFO("You are running playground!!!");
	ros::init(argc, argv, "nGPSlistener");
	
	ros::NodeHandle n;

	gps::initializeTrack();
	
	ros::Subscriber sub = n.subscribe("/gps/fix" , 1000, gps_callback);

	ros::spin();
	return 0;
}
