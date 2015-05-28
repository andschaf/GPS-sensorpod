#ifndef GPS_HANDLER_H_
#define GPS_HANDLER_H_

#include "geometry_msgs/Vector3.h"

const double kMajorAxis = 6378137; 
const double kMinorAxis	= 6356752.31;
const double kEarthRadius = (kMajorAxis+kMinorAxis)/2.0;  //TODO(andschaf): probably remove this.
const double kEccentricity = (kMajorAxis*kMajorAxis-kMinorAxis*kMinorAxis)/(kMajorAxis*kMajorAxis); 

#endif
