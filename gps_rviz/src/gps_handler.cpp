#include "gps_handler.h"

#include "geometry_msgs/Vector3.h"
#include "math.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

namespace gps {

 geometry_msgs::Vector3 InitializingPoint, initializing_point_enu_global;
 bool is_reference_frame_initialized;
 double lon_init, lat_init;
 
 visualization_msgs::MarkerArray gps_track;
 visualization_msgs::Marker gps_points, gps_line_strip, gps_line_list;

// Transform Earth-Centered-Earth-Fixed coordinates to East-North-Up coordinates.
void ecefToEnu(const geometry_msgs::Vector3& ecef, geometry_msgs::Vector3& enu) {
  enu.x = -sin(lon_init) * ecef.x + cos(lon_init) * ecef.y;
  enu.y = -cos(lon_init) * sin(lat_init) * ecef.x 
          - sin(lon_init) * sin(lat_init) * ecef.y 
          + cos(lat_init) * ecef.z;
  enu.z = cos(lon_init) * cos(lat_init) * ecef.x 
          + sin(lon_init) * cos(lat_init) * ecef.y 
          + sin(lat_init) * ecef.z;  
}

// Rotational Matrix plus shifting
void transformToMissionFrame(const geometry_msgs::Vector3& input_point, geometry_msgs::Vector3& gps_enu_mission) {
  // Rotate by alpha and beta
  geometry_msgs::Vector3 gps_enu_global;
  ecefToEnu(input_point, gps_enu_global); 
  // Shift by InitializingPoint in order to normalize
  gps_enu_mission.x = gps_enu_global.x - initializing_point_enu_global.x; 
  gps_enu_mission.y = gps_enu_global.y - initializing_point_enu_global.y;
  gps_enu_mission.z = gps_enu_global.z - initializing_point_enu_global.z;
} 

// Transform spherical coordinates [degree] to cartesian [WGS84, meters] coordinates
void sphericalToCartesian(const geometry_msgs::Vector3& gps_spherical,
   geometry_msgs::Vector3& gps_cartesian) {
  double earth_radius, height, lat_rad, lon_rad; 
   
  // Convert from degree to radian
  lat_rad = gps_spherical.x / 180.0 * M_PI; 	
  lon_rad = gps_spherical.y / 180.0 * M_PI; 
  height = gps_spherical.z;
 
  // Using WGS84 convention.
  earth_radius = kMajorAxis / sqrt(1.0 - kEccentricity * kEccentricity * sin(lat_rad) * sin(lat_rad));
  gps_cartesian.x = (earth_radius + height) * cos(lat_rad) * cos(lon_rad);
  gps_cartesian.y = (earth_radius + height) * cos(lat_rad) * sin(lon_rad);
  gps_cartesian.z = (earth_radius * (1.0 - kEccentricity * kEccentricity) + height) * sin(lat_rad);
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
