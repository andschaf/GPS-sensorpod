#include "gps_handler.h"

#include "math.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace gps {

 geometry_msgs::Vector3 InitializingPoint, initializing_point_mission_frame;
 bool is_reference_frame_initialized;
 float alpha_init, beta_init;
 
 visualization_msgs::MarkerArray gps_track;
 visualization_msgs::Marker gps_points, gps_line_strip, gps_line_list;

// Rotational Matrix R_0G=R_01*R_1G; R_01=Rz(alpha); R_1G=Ry(beta)
geometry_msgs::Vector3 transformation(geometry_msgs::Vector3 input, float alpha, float beta) {
  geometry_msgs::Vector3 output;
  output = input; // TODO(andschaf): for testing of the framework
	// TODO(andschaf): build inverse!

return output;
}

// Rotational Matrix plus shifting
void transformToMissionFrame(const geometry_msgs::Vector3& input_point, geometry_msgs::Vector3& transformed_point) {
  // Rotate by alpha and beta
  transformed_point = transformation(input_point, alpha_init, beta_init); 
  // Shift by InitializingPoint in order to normalize
  transformed_point.x = transformed_point.x - initializing_point_mission_frame.x; 
  transformed_point.y = transformed_point.y - initializing_point_mission_frame.y;
  transformed_point.z = transformed_point.z - initializing_point_mission_frame.z;
} 

// Transform spherical coordinates [degree] to cartesian [WGS84, meters] coordinates
void gpsToCartesian(const geometry_msgs::Vector3& gps_spherical,
   geometry_msgs::Vector3& gps_cartesian) {
  float height, phi_deg, phi_rad, lambda_deg, lambda_rad; 

  height = gps_spherical.z; 	
  phi_deg = gps_spherical.x; 	
  lambda_deg = gps_spherical.y; 	
  phi_rad = phi_deg / 180.0 * M_PI; 	
  lambda_rad = lambda_deg / 180.0 * M_PI; 	
  
  // Using WGS84 convention.
  gps_cartesian.x = (kEarthRadius + height) * cos(phi_rad) * cos(lambda_rad);
  gps_cartesian.y = (kEarthRadius + height) * cos(phi_rad) * sin(lambda_rad);
  gps_cartesian.z = (kEarthRadius * (1.0 - kEccentricity * kEccentricity) + height) * sin(phi_rad);

  ROS_INFO("In cartesian coordinates this is: [%f]||[%f]||[%f]", gps_cartesian.x, gps_cartesian.y,gps_cartesian.z);
}

// ----------------------------------------------
// Add a point to the track to be displayed in rviz as a MarkerArray.
void addToTrack(const geometry_msgs::Vector3& point_to_be_added_to_track) {
  geometry_msgs::Point point;
  point.x = point_to_be_added_to_track.x;
  point.y = point_to_be_added_to_track.y;
  point.z = point_to_be_added_to_track.z;

  gps_points.points.push_back(point);
  gps_line_strip.points.push_back(point);
  gps_line_list.points.push_back(point);
  point.z += 1.0;
  gps_line_list.points.push_back(point);

  gps_track.markers.push_back(gps_points);
  gps_track.markers.push_back(gps_line_strip);
  gps_track.markers.push_back(gps_line_list);
  ROS_INFO("Adding point [%f],[%f], [%f] in mission frame to Track.", 
   point_to_be_added_to_track.x, 
   point_to_be_added_to_track.y, 
   point_to_be_added_to_track.z); 	
}

// ------------------------------------------------------------
// Define the displaying parameters of the track for rviz.
void initializeTrack() {
  gps_points.header.frame_id = "map";
  gps_line_strip.header.frame_id = "map";
  gps_line_list.header.frame_id = "map"; 
  gps_points.header.stamp = ros::Time::now();
  gps_line_strip.header.stamp = ros::Time::now();
  gps_line_list.header.stamp = ros::Time::now();
  gps_points.ns = "gps_points_and_lines";
  gps_line_strip.ns = "gps_points_and_lines";
  gps_line_list.ns = "gps_points_and_lines";
  gps_points.action = visualization_msgs::Marker::ADD;
  gps_line_strip.action = visualization_msgs::Marker::ADD;
  gps_line_list.action = visualization_msgs::Marker::ADD;
  gps_points.pose.orientation.w = 1.0;
  gps_line_strip.pose.orientation.w = 1.0;
  gps_line_list.pose.orientation.w = 1.0; 

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

} // namespace gps
