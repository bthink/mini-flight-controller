#ifndef _AIRCRAFT_TRACKER_H_
#define _AIRCRAFT_TRACKER_H_

#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    bool valid;
    char callsign[16];
    float distance_km;
    int altitude_m;
    int heading_deg;
} aircraft_info_t;

esp_err_t aircraft_tracker_fetch_nearest(aircraft_info_t *out_info);

#endif // _AIRCRAFT_TRACKER_H_
