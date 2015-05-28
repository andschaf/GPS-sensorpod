#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"
#include "gps_handler.cpp"
#include "gps_rviz.h"
#define PI 3.14159265


void publishGPS_Track(){

	ros::NodeHandle node_handle_; 
	ros::Publisher gps_pub = node_handle_.advertise<visualization_msgs::MarkerArray>("GPS_marker",1000);
	gps_pub.publish(gps::GPS_TRACK);
	ros::spin();
}

void gps_callback(const sensor_msgs::NavSatFix gps_NavSatFix){ //callback function

	geometry_msgs::Vector3 gpsOut;

	gpsOut.x = gps_NavSatFix.latitude; // TODO this is sort of a hack...
	gpsOut.y = gps_NavSatFix.longitude; // x,y,z is misleading
	gpsOut.z = gps_NavSatFix.altitude;
	
	geometry_msgs::Vector3 GPS_point_WGS84;
	GPS_point_WGS84 = gps::gps2cart(gpsOut);	

	if (!gps::REFERENCE_FRAME_IS_INITIALIZED){
	 ROS_INFO("Initializing Reference Frame...");
	 gps::InitializingPoint = GPS_point_WGS84;
	 gps::alpha_init = PI/2 + gps_NavSatFix.longitude/180*PI;
	 gps::beta_init = PI/2 - gps_NavSatFix.latitude/180*PI;
	 gps::REFERENCE_FRAME_IS_INITIALIZED = true;
	
	 gps::InitializingPoint_missionframe = gps::transformation(gps::InitializingPoint, gps::alpha_init, gps::beta_init);
	 ROS_INFO("InitializingPoint in missionframe shoud be [0][0][R]!");
	 ROS_INFO("InitializingPoint in missionframe is [%f],[%f],[%f]", gps::InitializingPoint_missionframe.x,gps::InitializingPoint_missionframe.y,gps::InitializingPoint_missionframe.z);
	 
	} //endif

	geometry_msgs::Vector3 GPS_point_missionframe;
	GPS_point_missionframe = gps::transformToMissionFrame(GPS_point_WGS84); 
	gps::addToTrack(GPS_point_missionframe);	
	publishGPS_Track(); 
} 

int main(int argc, char **argv)
{	


	ROS_INFO("You are running playground!!!");
	ros::init(argc, argv, "nGPSlistener");
	ros::NodeHandle node_handle_;
	
	gps::initializeTrack();

	ros::Subscriber sub = node_handle_.subscribe("/gps/fix" , 1000, gps_callback);

	ros::spin();
	return 0;
}


// TODO real data set, validate, check with google maps

