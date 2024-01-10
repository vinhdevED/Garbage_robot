/*
 * path_distance.c
 *
 *  Created on: Oct 30, 2023
 *      Author: trand
 */

#include "math.h"
#include "geometric.h"

#define Radius 6371.0 //Radius of the earth (Km)
#define DEGTORAD M_PI / 180
#define RADTODEG 180/M_PI



double Haversine_Distance(Coordinate coordinate1, Coordinate coordinate2) {

	double lat1 = coordinate1.latitude * DEGTORAD;
	double lon1 = coordinate1.longitude * DEGTORAD;
	double lat2 = coordinate2.latitude * DEGTORAD;
	double lon2 = coordinate2.longitude * DEGTORAD;
	double a = sin((lat2 - lat1) / 2) * sin((lat2 - lat1) / 2) + cos(lat1) * cos(lat2) * sin((lon2 - lon1) / 2) * sin((lon2 - lon1) / 2);

    return (2.0 * Radius * atan2(sqrt(a), sqrt(1.0 - a))*1000.0);


}

//Hàm tính toán góc hợp bởi phương bắc và một đoạn thẳng nối 2 điểm tọa độ
/*Azimuth là góc giữa hướng Bắc (định cố định) và hướng của một đối tượng hoặc vị trí cụ thể,
 * khi nhìn từ một điểm gốc.*/
double Azimuth_Angle(Coordinate coordinate1, Coordinate coordinate2){

	//double deltaLongitude = (coordinate2.longitude - coordinate1.longitude) * DEGTORAD;
	double lat1 = coordinate1.latitude * DEGTORAD;
	double lon1 = coordinate1.longitude * DEGTORAD;
	double lat2 = coordinate2.latitude * DEGTORAD;
	double lon2 = coordinate2.longitude * DEGTORAD;

	double deltaLongitude = sin(lon2 - lon1)*cos(lat2);
	double deltaLatitude = cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2 - lon1);

	return atan2(deltaLongitude,deltaLatitude)*RADTODEG;

	//return atan2(sin(deltaLongitude)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(deltaLongitude)) * RADTODEG; //return in degree
}

float convertRange360(double deg) {
	if(deg < 0)
		return deg+360;
	return deg;
}

