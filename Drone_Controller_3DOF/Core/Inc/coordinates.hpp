/*
 * coordinates.h
 *
 *  Created on: Jul 21, 2022
 *      Author: Kerim
 */

#include <math.h>

#define Nlla 6.3781e6
#define e 0.0066

#ifndef INC_COORDINATES_H_
#define INC_COORDINATES_H_

void lla2ecef(float lla[3], float ecef[3]);

void ecef2ned(float ecef[3], float ecef0[3], float lla0[3], float vned[2]);




#endif /* INC_COORDINATES_H_ */
