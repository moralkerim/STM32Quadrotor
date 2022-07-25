/*
 * NMEA.h
 *
 *  Created on: 01-Jul-2022
 *  Author: Bader Ayran
 */

#ifndef INC_NMEA_H_
#define INC_NMEA_H_



#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "UartRingbuffer.h"

typedef struct {
	int hour;
	int min;
	int sec;
}TIME;

typedef struct {
	float latitude;
	char NS;
	float longitude;
	char EW;
}LOCATION;

typedef struct {
	float altitude;
	char unit;
}ALTITUDE;

typedef struct {
	int Day;
	int Mon;
	int Yr;
}DATE;

typedef struct {
	LOCATION lcation;
	TIME tim;
	int isfixValid;
	ALTITUDE alt;
	int numofsat;
	float HDOP;
}GGASTRUCT;

typedef struct {
	DATE date;
	float speed;
	float course;
	int isValid;
}RMCSTRUCT;

typedef struct {
	GGASTRUCT ggastruct;
	RMCSTRUCT rmcstruct;
}GPSSTRUCT;

extern unsigned long gga_time;
extern unsigned long gga_time_dif;

char *getDataAt (char * str, const char * delim, int pos);

int decodeGGA (char *GGAbuffer, GGASTRUCT *gga);

int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc);

void getGPSData (GPSSTRUCT *gpsData);

double pow2(long number, int power);
//float pow(float number, int power);

#endif /* INC_NMEA_H_ */
