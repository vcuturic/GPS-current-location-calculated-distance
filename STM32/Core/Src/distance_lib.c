#include "distance_lib.h"
#include "math.h"
#include "stdlib.h"

#define EARTH_RADIUS_KM 6371.0
#define TO_RADIANS(deg) ((deg) * M_PI / 180.0)

double haversineDistance(double lat_1, double lon_1, double lat_2, double lon_2) {
    // Convert latitude and longitude from degrees to radians
    lat_1 = TO_RADIANS(lat_1);
    lon_1 = TO_RADIANS(lon_1);
    lat_2 = TO_RADIANS(lat_2);
    lon_2 = TO_RADIANS(lon_2);

    // Calculate the haversine distance
    double dlat = lat_2 - lat_1;
    double dlon = lon_2 - lon_1;

    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(lat_1) * cos(lat_2) * sin(dlon / 2.0) * sin(dlon / 2.0);

    double c = 2.0 * atan2(sqrt(a), sqrt(1 - a));

    double distance = EARTH_RADIUS_KM * c;

    return distance;
}
