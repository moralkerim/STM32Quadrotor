/*
 * NMEA.c
 *
 *  Created on: 01-Jul-2022
 *  Author: Bader Ayran
 */

/****** TO DO:
 * IMPLEMENT DECODING FOR MULTIPLE PACKAGES
 * MAKE SURE NO HARD FAULT ERRORS OCCUR
******/

#include "NMEA.h"

int GMT = +300;

int inx = 0;
int hr=0,min=0,day=0,mon=0,yr=0;
int daychange = 0;

/* Extracts a specific data point from a string of data separated with a delimiter
   @str is the buffer which stores the data
   @delim is the delimiter
   @pos is the position of data (starts from 0)
   @returns the specified data as a string
*/

char *getDataAt (char * str, const char * delim, int pos) {
	char *token;
	char copy[100] = "\0"; // modify size for larger strings

	strcpy(copy, str);
	token = strtok(copy, delim);
	for(int i = 0; i < pos; i++) {
		token = strtok(NULL, delim);
	}
	return token;
}


/* Decodes the GGA Data
   @GGAbuffer is the buffer which stores the GGA Data
   @GGASTRUCT is the pointer to the GGA Structure (in the GPS Structure)
   @Returns 0 on success
   @ returns 1, 2 depending on where the return statement is excuted, check function for more details
*/

int decodeGGA (char *GGAbuffer, GGASTRUCT *gga) {

	char buffer[12];

	strcpy(buffer, getDataAt(GGAbuffer, ",", 5));
	if (buffer[0] == '1' || buffer[0] == '2' || buffer[0] == '6')   // 0 indicates no fix yet
	{
		gga->isfixValid = 1;   // fix available
	}
	else
	{
		gga->isfixValid = 0;   // If the fix is not available
		return 1;  // return error
	}


	/*********************** Get TIME ***************************/
	//(Update the GMT Offset at the top of this file)

	memset(buffer, '\0', 12);
	strcpy(buffer, getDataAt(GGAbuffer, ",", 0));

	hr = (atoi(buffer)/10000) + GMT/100;   // get the hours from the 6 digit number

	min = ((atoi(buffer)/100)%100) + GMT%100;  // get the minutes from the 6 digit number

	// adjust time
	if (min > 59) {
		min = min-60;
		hr++;
	}
	if (hr<0) {
		hr=24+hr;
		daychange--;
	}
	if (hr>=24) {
		hr=hr-24;
		daychange++;
	}

	// Store the time in the GGA structure
	gga->tim.hour = hr;
	gga->tim.min = min;
	gga->tim.sec = atoi(buffer)%100;

	/***************** Get LATITUDE  **********************/

	memset(buffer, '\0', 12);
	strcpy(buffer, getDataAt(GGAbuffer, ",", 1));

	if (strlen(buffer) < 6) return 2;  	// If the buffer length is not appropriate, return error

	int16_t num = (atoi(buffer));   	// change the buffer to a number. It will only convert up to decimal
	int dd = num/100;					// extract the degrees
	int mmint = num%100;				// extract the integer part of minutes
	int j = 0;
	while (buffer[j] != '.') {
		if(j > strlen(buffer)) return 2;
		j++;   	// Figure out how many digits before the decimal
	}
	j++;
	int declen = (strlen(buffer))-j;  	// calculate the number of digit after decimal
	int mmdec = atoi ((char *) buffer+j);  // extract the decimal part of minutes
	float lat = dd + (mmint + mmdec/pow(10, (declen)))/60;	// combine minutes and convert to degrees
	gga->lcation.latitude = lat;  		// save the latitude data into the structure

	gga->lcation.NS = *getDataAt(GGAbuffer, ",", 2);  // save the N/S into the structure


	/***********************  GET LONGITUDE **********************/

	memset(buffer, '\0', 12);
	strcpy(buffer, getDataAt(GGAbuffer, ",", 3));

	num = (atoi(buffer));  	// change the buffer to the number. It will only convert up to decimal
	dd = num/100;			// extract the degrees
	mmint = num%100;		// extract the integer part of minutes
	j = 0;
	while (buffer[j] != '.') {
		if(j > strlen(buffer)) return 2;
		j++;   	// Figure out how many digits before the decimal
	}
	j++;
	declen = (strlen(buffer))-j;  		// calculate the number of digit after decimal
	mmdec = atoi ((char *) buffer+j);  	// extract the decimal part of minutes
	lat = dd + (mmint + mmdec/pow(10, (declen)))/60;  // combine minutes and convert to degrees
	gga->lcation.longitude = lat;  // save the longitude data into the structure

	gga->lcation.EW = *getDataAt(GGAbuffer, ",", 4);  // save the E/W into the structure

	/***************** NUMMBER OF SATELLITES  *********************/

	strcpy(buffer, getDataAt(GGAbuffer, ",", 6));

	gga->numofsat = atoi(buffer);   // convert the buffer to number and save into the structure

	/***************** HDOP  *********************/

	memset(buffer, '\0', 12);
	strcpy(buffer, getDataAt(GGAbuffer, ",", 7));

	num = (atoi(buffer));
	j = 0;
	while (buffer[j] != '.') {
		if(j > strlen(buffer)) return 2;
		j++;   	// Figure out how many digits before the decimal
	}
	j++;
	declen = (strlen(buffer))-j;
	int dec = atoi ((char *) buffer+j);
	lat = (num) + (dec/pow(10, (declen)));
	gga->HDOP = lat;

	/*************** ALTITUDE CALCULATION ********************/

	memset(buffer, '\0', 12);
	strcpy(buffer, getDataAt(GGAbuffer, ",", 8));

	num = (atoi(buffer));
	j = 0;
	while (buffer[j] != '.') {
		if(j > strlen(buffer)) return 2;
		j++;   	// Figure out how many digits before the decimal
	}
	j++;
	declen = (strlen(buffer))-j;
	dec = atoi ((char *) buffer+j);
	lat = (num) + (dec/pow(10, (declen)));
	gga->alt.altitude = lat;

	gga->alt.unit = *getDataAt(GGAbuffer, ",", 9);

	return 0;

}


int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc) {

	char buffer[12];

	if (*getDataAt(RMCbuffer, ",", 1) == 'A') rmc->isValid = 1; // Here 'A' Indicates the data is valid, and 'V' indicates invalid data
	else {
		rmc->isValid = 0;
		return 1;
	}

	memset(buffer, '\0', 12);
	strcpy(buffer, getDataAt(RMCbuffer, ",", 6));

	int j = 0;
	while (buffer[j] != '.') {
		if(j > strlen(buffer)) return 2;
		j++;   // same as above
	}

	if (strlen (buffer) > j) {          // if the speed have some valid data
		int16_t num = (atoi(buffer));  // convert the data into the number
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->speed = lat;
	}
	else rmc->speed = 0;

	// Get Course

	memset(buffer, '\0', 12);
	strcpy(buffer, getDataAt(RMCbuffer, ",", 7));

	j = 0;
	while (buffer[j] != '.') {
		if(j > strlen(buffer)) return 2;
		j++;   	// Figure out how many digits before the decimal
	}

	if (strlen (buffer) > j){  // if the course have some data
		int16_t num = (atoi(buffer));   // convert the course data into the number
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->course = lat;
	}
	else rmc->course = 0;


	// Get Date

	memset(buffer, '\0', 12);
	if (rmc->course == 0) strcpy(buffer, getDataAt(RMCbuffer, ",", 7));
	else strcpy(buffer, getDataAt(RMCbuffer, ",", 8));

	if (strlen(buffer) == 6){
		// Date in the format 070722
		day = atoi(buffer)/10000;  // extract 28
		mon = (atoi(buffer)/100)%100;  // extract 02
		yr = atoi(buffer)%100;  // extract 22

		day = day+daychange;   // correction due to GMT shift

		// save the data into the structure
		rmc->date.Day = day;
		rmc->date.Mon = mon;
		rmc->date.Yr = yr;
	}
	return 0;
}

void getGPSData (GPSSTRUCT *gpsData) {
	char GGA[100];
	//char RMC[100];


	if (Wait_for("GGA")) {
		Copy_upto("*", GGA);
		decodeGGA(GGA, &gpsData->ggastruct);
	}
/*
	if (Wait_for("RMC")) {
		Copy_upto("*", RMC);
		decodeRMC(RMC, &gpsData->rmcstruct);
	}
*/


	//Uart_flush();
	//return;

}
