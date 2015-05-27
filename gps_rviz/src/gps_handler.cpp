#include "geometry_msgs/Vector3.h"
#include "math.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "gps_handler.h"

#define PI 3.14159265

using namespace std;

namespace gps{

 geometry_msgs::Vector3 InitializingPoint, InitializingPoint_missionframe;
 bool REFERENCE_FRAME_IS_INITIALIZED;
 float alpha_init, beta_init;
 
 visualization_msgs::MarkerArray GPS_TRACK;
 visualization_msgs::Marker gps_points, gps_line_strip, gps_line_list;

// ---------------------------------------------
// Rotational Matrix R_0G=R_01*R_1G; R_01=Rz(a); R_1G=Ry(b)
geometry_msgs::Vector3 transformation(geometry_msgs::Vector3 Input, float a, float b){
	geometry_msgs::Vector3 Output;
 	Output.x = cos(a)*cos(b)*Input.x - sin(a)*Input.y +cos(a)*sin(b)*Input.z;
 	Output.y = sin(a)*cos(b)*Input.x + cos(a)*Input.y +sin(a)*sin(b)*Input.z;
 	Output.z = -sin(b)*Input.x + cos(b)*Input.z;

return Output;
}

// -------------------------------------------------
// Rotational Matrix plus shifting
geometry_msgs::Vector3 transformToMissionFrame(geometry_msgs::Vector3 InputPoint){

	geometry_msgs::Vector3 transformed_point;

	transformed_point = transformation(InputPoint, alpha_init, beta_init); //Rotate by alpha, beta
	transformed_point.x = transformed_point.x - InitializingPoint_missionframe.x; // shift to normalize
	transformed_point.y = transformed_point.y - InitializingPoint_missionframe.y;
	transformed_point.z = transformed_point.z - InitializingPoint_missionframe.z;

return transformed_point;
} 

// ---------------------------------------------------
// NavSatFix coordinates to WGS84 (cartesian) coordinates
geometry_msgs::Vector3 gps2cart(const geometry_msgs::Vector3& gpsOut){
	
	float e, R, height, phi_deg, phi_rad, lambda_deg, lambda_rad; 

	R = (majorAxis+minorAxis)/2; //in meters
	e = (majorAxis*majorAxis-minorAxis*minorAxis)/(majorAxis*majorAxis); //in meters

	height 		= gpsOut.z; 		//meters
	phi_deg		= gpsOut.x; 		//degrees
 	lambda_deg	= gpsOut.y; 		//degrees
	phi_rad 	= phi_deg/180*PI; 	//radians
	lambda_rad 	= lambda_deg/180*PI; 	//radians

	geometry_msgs::Vector3 gps_cart;
	gps_cart.x = (R+height)*cos(phi_rad)*cos(lambda_rad); // WGS84
	gps_cart.y = (R+height)*cos(phi_rad)*sin(lambda_rad);
	gps_cart.z = (R*(1-e*e)+height)*sin(phi_rad);

	ROS_INFO("In cartesian coordinates this is: [%f]||[%f]||[%f]", gps_cart.x, gps_cart.y,gps_cart.z);
return gps_cart;
}

// ----------------------------------------------
// Add a single point (x,y,z) to MarkerArray GPS_TRACK 
void addToTrack(const geometry_msgs::Vector3 PointToBeAddedToTrack){

	geometry_msgs::Point p;
	p.x = PointToBeAddedToTrack.x;
	p.y = PointToBeAddedToTrack.y;
	p.z = PointToBeAddedToTrack.z;

	gps_points.points.push_back(p);
	gps_line_strip.points.push_back(p);
	gps_line_list.points.push_back(p);
	p.z += 1.0;
	gps_line_list.points.push_back(p);

	GPS_TRACK.markers.push_back(gps_points);
	GPS_TRACK.markers.push_back(gps_line_strip);
	GPS_TRACK.markers.push_back(gps_line_list);
	ROS_INFO("Adding point [%f],[%f], [%f] in mission frame to Track.", PointToBeAddedToTrack.x,PointToBeAddedToTrack.y,PointToBeAddedToTrack.z); 	
}

// ------------------------------------------------------------
// Define parameters of GPS_TRACK
void initializeTrack(){
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
}

// ----------------------------------------------------------------------
} //end namespace gps
