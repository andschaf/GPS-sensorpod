#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "math.h"

#define PI 3.14159265

//----------------------------------------
geometry_msgs::Vector3 gps2cart(const geometry_msgs::Vector3& gpsOut)
{	
	float majorAxis;
	float minorAxis;
	float e; 
	float R;


	majorAxis 	= 6378137; //in meters
	minorAxis 	= 6356752.31; //in meters
	R 		= (majorAxis+minorAxis)/2; //in meters
	e = (majorAxis*majorAxis-minorAxis*minorAxis)/(majorAxis*majorAxis); //in meters

	ROS_INFO("You defined the major Axis to be [%f]m, the minor axis [%f]m.",majorAxis, minorAxis);	
	ROS_INFO("With that I calculated the radius to be [%f]m and e is [%f]m.", R, e);

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


void publishGPSpositions(const geometry_msgs::Vector3 msg){
	visualization_msgs::Marker gps_points, gps_line_strip, gps_line_list;

	ROS_INFO("I defined gps_points, gps_line_strip, gps_line_list");
	
	ros::NodeHandle n;
	ros::Publisher gps_pub = n.advertise<visualization_msgs::Marker>("GPS_marker",1000);

	ROS_INFO("Advertising GPS_marker...");
	if (ros::ok()) {

		gps_points.header.frame_id = gps_line_strip.header.frame_id = gps_line_list.header.frame_id = "map"; 
		gps_points.header.stamp = gps_line_strip.header.stamp = gps_line_list.header.stamp = ros::Time::now();
		gps_points.ns = gps_line_strip.ns =gps_line_list.ns = "gps_points_and_lines";
		gps_points.action = gps_line_strip.action = gps_line_list.action = visualization_msgs::Marker::ADD;
		gps_points.pose.orientation.w = gps_line_strip.pose.orientation.w = gps_line_list.pose.orientation.w = 1.0; //other components of pose are set to default=0

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

		geometry_msgs::Point p;
		p.x = msg.x-4266066; //TODO Normalize around first entry point
		p.y = msg.y-584902;
		p.z = msg.z-4691134;

		gps_points.points.push_back(p);
		gps_line_strip.points.push_back(p);
		gps_line_list.points.push_back(p);
		p.z += 1.0;
		gps_line_list.points.push_back(p);

		int size_gps_points;
		size_gps_points=gps_points.points.size();

		int size_gps_line_strip;
		size_gps_line_strip=gps_line_strip.points.size();

		int size_gps_line_list;
		size_gps_line_list=gps_line_list.points.size();

		ROS_INFO("size_gps_points= [%d]; size_gps_line_strip= [%d]; size_gps_line_list= [%d]", 	size_gps_points,size_gps_line_strip,size_gps_line_list);

		ROS_INFO("Trying to publish...");
		gps_pub.publish(gps_points);
		gps_pub.publish(gps_line_strip);
		gps_pub.publish(gps_line_list);
	
		ROS_INFO("Published successfully.");

		ros::spin();

	} //end while ros::ok()
	
}

void gps2rviz(const sensor_msgs::NavSatFix gps_NavSatFix){ //callback function

	geometry_msgs::Vector3 gpsOut;

	gpsOut.x = gps_NavSatFix.latitude; 
	gpsOut.y = gps_NavSatFix.longitude;
	gpsOut.z = gps_NavSatFix.altitude;

	ROS_INFO("I extracted: lat=[%f] , lon=[%f] and alt=[%f]" ,gpsOut.x, gpsOut.y, gpsOut.z);
	
	geometry_msgs::Vector3 cleaned_GPS_point;
	cleaned_GPS_point = gps2cart(gpsOut);	
	
	ROS_INFO("in gps2rviz I return [%f]||[%f]||[%f]", cleaned_GPS_point.x, cleaned_GPS_point.y, cleaned_GPS_point.z);
	publishGPSpositions(cleaned_GPS_point);
} 

// -------------------------------------------

int main(int argc, char **argv)
{	

	ros::init(argc, argv, "nGPSlistener");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/gps/fix" , 1000, gps2rviz);

	ros::spin();
	return 0;
}
