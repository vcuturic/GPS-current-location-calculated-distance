/*
 * nmea_parse.c
 *
 *  Created on: Aug 1, 2023
 *      Author: vcutu
 */

#include "nmea_parse.h"
#include "ftoa.h"
#include "string.h"
#include "stdlib.h"

#define IRRELEVANT 0
#define TIME 1
#define LATITUDE 2
#define NORTH_SOUTH 3
#define LONGITUDE 4
#define EAST_WEST 5
#define FIX 6
#define MAX_GGA_SIZE 100

extern int charToNumber(char c);
extern float strToFloat(char *str);

int parse_GPGGA(char *GGAbuffer, GGASTRUCT *ggaStruct) {
	int currentData = 0;

	//"$GPGGA,093121.691,5404.2675,N,00159.7569,W,1,10,4.00,100.0,M,S0.0,M„*7A\r\n",
	char *token = strtok(GGAbuffer, ",");
	// token = $GPGGA 093121.691 5404.2675 N 00159.7569 W 1 10 4.00 100.0 M
	// irelevant, time, latitude NS longitude EW fix
	char *marker = token;

	while (marker != NULL) {
		if(currentData == TIME)
			ggaStruct->time = strToFloat(marker);
		if(currentData == LATITUDE)
			ggaStruct->latitude = strToFloat(marker);
		if(currentData == NORTH_SOUTH)
			ggaStruct->northSouth = charToNumber((char)marker[0]);
		if(currentData == LONGITUDE)
			ggaStruct->longitude = strToFloat(marker);
		if(currentData == EAST_WEST)
			ggaStruct->eastWest = (char)marker[0];
		if(currentData == FIX)
			ggaStruct->fix = charToNumber((char)marker[0]);

		marker = strtok(NULL, ",");
		currentData++;
	}

	return 1;
}

char *prepareData(char *nmea_data) {
	int commmaCounter = 0;
	int idx = 0;

	char *str = (char*)malloc(MAX_GGA_SIZE * sizeof(char));
	strcpy(str, nmea_data);

	while(commmaCounter < 7) {
		if(str[idx] == ',')
			commmaCounter++;
		idx++;
	}

	str[--idx] = '\0';

	return str;
}

void nmea_parse(char *gps_data[], GPSSTRUCT *gpsStruct) {
	int dataLen = 3;

	for (int i = 0; i < dataLen; i++) {

		if (strstr(gps_data[i], "\r\n") != NULL) {
			if (strstr(gps_data[i], "GPGGA") != NULL) {
				char *str = prepareData(gps_data[i]);
				parse_GPGGA(str, &gpsStruct->ggaStruct);
			}
//			else if (strstr(gps_data[i], "GPRMC") != NULL) {
//				parse_GPRMC(gps_data, gpsStruct->rmcStruct);
//			}
		}
	}
}
