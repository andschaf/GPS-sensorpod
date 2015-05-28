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
  geometry_msgs::Vector3 gps_spherical_degree;
  gps_spherical_degree.x = gps_NavSatFix.latitude;
  gps_spherical_degree.y = gps_NavSatFix.longitude;
  gps_spherical_degree.z = gps_NavSatFix.altitude;

  ROS_INFO("Spherical coordinates degree: [%f],[%f],[%f]", 
   gps_spherical_degree.x, gps_spherical_degree.y, gps_spherical_degree.z);

  geometry_msgs::Vector3 gps_point_ecef;
  gps::sphericalToCartesian(gps_spherical_degree, gps_point_ecef);	
  
  // Define Reference Frame if not done yet. 
  if (!gps::is_reference_frame_initialized){
   ROS_INFO("Initializing Reference Frame...");
   gps::InitializingPoint = gps_point_ecef;
   gps::lon_init = gps_NavSatFix.longitude / 180.0 * M_PI;
   gps::lat_init = gps_NavSatFix.latitude / 180.0 * M_PI;
   gps::is_reference_frame_initialized = true;

   ROS_INFO("lon_init = [%f], lat_init = [%f]", gps::lon_init, gps::lat_init);
	
   gps::ecefToEnu(gps::InitializingPoint, gps::initializing_point_enu_global);	 
  } //if

  geometry_msgs::Vector3 GPS_point_missionframe;
  gps::transformToMissionFrame(gps_point_ecef, GPS_point_missionframe); 
  gps::addToTrack(GPS_point_missionframe);	
  publishGPS_Track(); 
} 

int main(int argc, char **argv)
{	
  ros::init(argc, argv, "nGPSlistener");
  ros::NodeHandle node_handle;
	
  gps::initializeTrack();

  ros::Subscriber sub = node_handle.subscribe("/gps/fix" , 1000, gpsCallback);
  ros::spin();
  return 0;
}
