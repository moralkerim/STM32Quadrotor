/*
 * coordinates.c
 *
 *  Created on: Jul 21, 2022
 *      Author: Kerim
 */
#include <coordinates.hpp>

void lla2ecef(float lla[3], float ecef[3]) {
	float deg2rad = M_PI/180.0;
	float lat = deg2rad*lla[0];
	float lon = deg2rad*lla[1];
	float h = lla[2];

	ecef[0] = (Nlla+h)*cos(lat)*cos(lon);
	ecef[1] = (Nlla+h)*cos(lat)*sin(lon);
	ecef[2] = ((1-e*e)*Nlla+h)*sin(lat);
/*
	ecef[0] = 1;
	ecef[1] = 2;
	ecef[2] = 4;
	*/
}

void ecef2ned(float ecef[3], float ecef0[3], float lla0[3], float vned[2]) {
	float dxecef = ecef[0]- ecef0[0];
	float dyecef = ecef[1]- ecef0[1];
	float dzecef = ecef[2] -ecef0[2];

	float lat0 = lla0[0];
	float lon0 = lla0[1];

	vned[1]=  -sin(lon0)*dxecef + cos(lon0)*dyecef;
	vned[0] = -sin(lat0)*cos(lon0)*dxecef - sin(lat0)*sin(lon0)*dyecef + cos(lat0)*dzecef;
}
