#ifndef _AIRCRAFT_TRACKER_H_
#define _AIRCRAFT_TRACKER_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifdef __cplusplus
extern "C" {
#endif

// Aircraft data structure based on OpenSky Network response
typedef struct {
    char icao24[8];           // ICAO24 hex code (e.g., "484506")
    char callsign[10];        // Flight callsign (e.g., "LOT123")
    char origin_country[32];  // Country of origin
    float latitude;           // Latitude in decimal degrees
    float longitude;          // Longitude in decimal degrees
    float altitude;           // Altitude in meters (barometric)
    float velocity;           // Ground speed in m/s
    float heading;            // True track in degrees
    bool on_ground;           // Aircraft on ground flag
    uint32_t last_contact;    // Unix timestamp of last contact
    float distance_km;        // Calculated distance from our position
    uint8_t direction;        // Direction relative to us (0-7, for arrows)
    bool valid;               // Data validity flag
} aircraft_data_t;

// Aircraft tracker status
typedef enum {
    TRACKER_STATUS_DISABLED,
    TRACKER_STATUS_INITIALIZING,
    TRACKER_STATUS_RUNNING,
    TRACKER_STATUS_ERROR,
    TRACKER_STATUS_NO_WIFI
} tracker_status_t;

// Aircraft tracker statistics
typedef struct {
    uint32_t total_requests;     // Total API requests made
    uint32_t successful_requests; // Successful API requests
    uint32_t last_update_time;   // Last successful update timestamp
    uint8_t aircraft_count;      // Current number of tracked aircraft
    tracker_status_t status;     // Current tracker status
} tracker_stats_t;

/**
 * @brief Initialize aircraft tracker
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aircraft_tracker_init(void);

/**
 * @brief Start aircraft tracking task
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aircraft_tracker_start(void);

/**
 * @brief Stop aircraft tracking task
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aircraft_tracker_stop(void);

/**
 * @brief Get current aircraft data
 * 
 * @param aircraft_list Buffer to store aircraft data
 * @param max_count Maximum number of aircraft to retrieve
 * @param actual_count Pointer to store actual number of aircraft retrieved
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aircraft_tracker_get_aircraft(aircraft_data_t *aircraft_list, uint8_t max_count, uint8_t *actual_count);

/**
 * @brief Get tracker statistics
 * 
 * @param stats Pointer to store statistics
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aircraft_tracker_get_stats(tracker_stats_t *stats);

/**
 * @brief Force immediate update (for testing)
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t aircraft_tracker_force_update(void);

/**
 * @brief Check if tracker is running
 * 
 * @return bool true if running, false otherwise
 */
bool aircraft_tracker_is_running(void);

#ifdef __cplusplus
}
#endif

#endif // _AIRCRAFT_TRACKER_H_ 