#ifndef _AIRCRAFT_TRACKER_H_
#define _AIRCRAFT_TRACKER_H_

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool valid;
    char callsign[16];
    char registration[16];
    char aircraft_type_code[16];
    char aircraft_type_name[48];
    float distance_km;
    int altitude_m;
    int heading_deg;
    char origin_airport[8];
    char destination_airport[8];
} aircraft_info_t;

esp_err_t aircraft_tracker_fetch_nearest(aircraft_info_t *out_info);

#endif // _AIRCRAFT_TRACKER_H_
