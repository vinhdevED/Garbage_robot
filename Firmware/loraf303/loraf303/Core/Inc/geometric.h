/*
 * path_distance.h
 *
 *  Created on: Oct 30, 2023
 *      Author: trand
 */

#ifndef INC_GEOMETRIC_H_
#define INC_GEOMETRIC_H_

typedef struct {
	double latitude;
	double longitude;
} Coordinate;


double Haversine_Distance(Coordinate coordinate1, Coordinate coordinate2);
double Azimuth_Angle(Coordinate coordinate1, Coordinate coordinate2);
float convertRange360(double deg);

#endif /* INC_GEOMETRIC_H_ */
