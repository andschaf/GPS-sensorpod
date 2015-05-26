#include "geometry_msgs/Vector3.h"
#include "math.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "gps_handler.h"

#define PI 3.14159265

using namespace std;

namespace gps{

visualization_msgs::MarkerArray GPS_TRACK;
visualization_msgs::Marker gps_points, gps_line_strip, gps_line_list;
	
geometry_msgs::Vector3 gps2cart(const geometry_msgs::Vector3& gpsOut)
{	
	float e; 
	float R;

	R = (majorAxis+minorAxis)/2; //in meters
	e = (majorAxis*majorAxis-minorAxis*minorAxis)/(majorAxis*majorAxis); //in meters

	float height; 		//meters altitude
	float phi_deg; 		//degree, latitude
	float lambda_deg; 	//degree, longitude
	float phi_rad;
	float lambda_rad;

	height 		= gpsOut.z; 		//meters
	phi_deg		= gpsOut.x; 		//degrees
 	lambda_deg	= gpsOut.y; 		//degrees
	phi_rad 	= phi_deg/180*PI; 	//radians
	lambda_rad 	= lambda_deg/180*PI; 	//radians

	geometry_msgs::Vector3 gps_cart;
	gps_cart.x = (R+height)*cos(phi_rad)*cos(lambda_rad); //TODO introduce WGS84!!
	gps_cart.y = (R+height)*cos(phi_rad)*sin(lambda_rad);
	gps_cart.z = (R*(1-e*e)+height)*sin(phi_rad);

	ROS_INFO("In cartesian coordinates this is: [%f]||[%f]||[%f]", gps_cart.x, gps_cart.y,gps_cart.z);
return gps_cart;
}


void addToTrack(const geometry_msgs::Vector3 PointToBeAddedToTrack){

	geometry_msgs::Point p;
	p.x = PointToBeAddedToTrack.x-InitializingPoint_x;
	p.y = PointToBeAddedToTrack.y-InitializingPoint_y;
	p.z = PointToBeAddedToTrack.z-InitializingPoint_z;

	gps_points.points.push_back(p);
	gps_line_strip.points.push_back(p);
	gps_line_list.points.push_back(p);
	p.z += 1.0;
	gps_line_list.points.push_back(p);

	GPS_TRACK.markers.push_back(gps_points);
	GPS_TRACK.markers.push_back(gps_line_strip);
	GPS_TRACK.markers.push_back(gps_line_list);
}

void initializeTrack(){

// Define all the parameters
	gps_points.header.frame_id = gps_line_strip.header.frame_id = gps_line_list.header.frame_id = "map"; 
	gps_points.header.stamp = gps_line_strip.header.stamp = gps_line_list.header.stamp = ros::Time::now();
	gps_points.ns = gps_line_strip.ns =gps_line_list.ns = "gps_points_and_lines";
	gps_points.action = gps_line_strip.action = gps_line_list.action = visualization_msgs::Marker::ADD;
	gps_points.pose.orientation.w = gps_line_strip.pose.orientation.w = gps_line_list.pose.orientation.w = 1.0; 

	gps_points.id = 0;
	gps_line_strip.id = 1;
	gps_line_list.id = 2;

	gps_points.type = visualization_msgs::Marker::POINTS;
	gps_line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	gps_line_list.type = visualization_msgs::Marker::LINE_LIST;

	gps_points.scale.x = 0.2;
	gps_points.scale.y = 0.2;
	gps_line_strip.scale.x = 0.1;
	gps_line_list.scale.x = 0.1;
	gps_points.color.g = 1.0f;
	gps_points.color.a = 1.0;
	gps_line_strip.color.b = 1.0;
	gps_line_strip.color.a = 1.0;	
	gps_line_list.color.r = 1.0;
	gps_line_list.color.a = 1.0;

	ROS_INFO("Defined type, scale and color successfully");

	geometry_msgs::Vector3 InitializingPoint;
	InitializingPoint.x = InitializingPoint_x;
	InitializingPoint.y = InitializingPoint_y;
	InitializingPoint.z = InitializingPoint_z;
	
	addToTrack(InitializingPoint);
	ROS_INFO("Initialized Track successfully. First point: [%f],[%f],[%f].", InitializingPoint.x, InitializingPoint.y, InitializingPoint.z);

} //end initializeTrack()

} //end namespace gps
