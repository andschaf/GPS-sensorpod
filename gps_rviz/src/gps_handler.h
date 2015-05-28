#ifndef GPS_HANDLER_H_
#define GPS_HANDLER_H_

#include "geometry_msgs/Vector3.h"

const float kMajorAxis = 6378137; 
const float kMinorAxis 	= 6356752.31;
const float kEarthRadius = (kMajorAxis+kMinorAxis)/2.0; 
const float kEccentricity = (kMajorAxis*kMajorAxis-kMinorAxis*kMinorAxis)/(kMajorAxis*kMajorAxis); 

#endif
