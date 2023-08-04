/*
 * nmea_parse.h
 *
 *  Created on: Aug 1, 2023
 *      Author: vcutu
 */

#ifndef INC_NMEA_PARSE_H_
#define INC_NMEA_PARSE_H_

typedef struct {
	float time;
	float latitude;
	char northSouth;
	float longitude;
	char eastWest;
	int fix;
} GGASTRUCT;

typedef struct {
	float time;
	float latitude;
	char northSouth;
	float longitude;
	char eastWest;
	int fix;
} RMCSTRUCT;

typedef struct {
	GGASTRUCT ggaStruct;
	RMCSTRUCT rmcStruct;
}GPSSTRUCT;

void nmea_parse(char *gps_data[], GPSSTRUCT *gpsStruct);

#endif /* INC_NMEA_PARSE_H_ */
