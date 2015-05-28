#include "gps_rviz.h"

#include "geometry_msgs/Vector3.h"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

#include "gps_handler.cpp"

void publishGPS_Track() {
  ros::NodeHandle node_handle; 
  ros::Publisher gps_pub = 
   node_handle.advertise<visualization_msgs::MarkerArray>("GPS_marker", 1000);
  gps_pub.publish(gps::gps_track);
  ros::spin();
}

void gpsCallback(const sensor_msgs::NavSatFix& gps_NavSatFix) {
  geometry_msgs::Vector3 gps_spherical;
  gps_spherical.x = gps_NavSatFix.latitude; // TODO(andschaf): this is sort of a hack...
  gps_spherical.y = gps_NavSatFix.longitude; // x,y,z is misleading
  gps_spherical.z = gps_NavSatFix.altitude;

  geometry_msgs::Vector3 gps_point_wgs84;
  gps::gpsToCartesian(gps_spherical, gps_point_wgs84);	

  if (!gps::is_reference_frame_initialized){
   ROS_INFO("Initializing Reference Frame...");
   gps::InitializingPoint = gps_point_wgs84;
   gps::alpha_init = M_PI / 2.0 + gps_NavSatFix.longitude / 180.0 * M_PI;
   gps::beta_init = M_PI / 2.0 - gps_NavSatFix.latitude / 180.0 * M_PI;
   gps::is_reference_frame_initialized = true;
	
   gps::initializing_point_mission_frame = 
   gps::transformation(gps::InitializingPoint, gps::alpha_init, gps::beta_init);
   ROS_INFO("InitializingPoint in missionframe shoud be [0][0][R]!");
   ROS_INFO("InitializingPoint in missionframe is [%f],[%f],[%f]", 
    gps::initializing_point_mission_frame.x,  
    gps::initializing_point_mission_frame.y,
    gps::initializing_point_mission_frame.z);
	 
  } //if

  geometry_msgs::Vector3 GPS_point_missionframe;
  gps::transformToMissionFrame(gps_point_wgs84, GPS_point_missionframe); 
  gps::addToTrack(GPS_point_missionframe);	
  publishGPS_Track(); 
} 

int main(int argc, char **argv)
{	
  ROS_INFO("You are running playground!!!");
  ros::init(argc, argv, "nGPSlistener");
  ros::NodeHandle node_handle;
	
  gps::initializeTrack();

  ros::Subscriber sub = node_handle.subscribe("/gps/fix" , 1000, gpsCallback);

  ros::spin();
  return 0;
}


// TODO(andschaf): real data set, validate, check with google maps

