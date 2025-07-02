#ifndef _WIFI_MANAGER_H_
#define _WIFI_MANAGER_H_

#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief WiFi connection status
 */
typedef enum {
    WIFI_STATUS_DISCONNECTED,
    WIFI_STATUS_CONNECTING,
    WIFI_STATUS_CONNECTED,
    WIFI_STATUS_FAILED
} wifi_status_t;

/**
 * @brief Initialize WiFi and connect to configured network
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief Get current WiFi connection status
 * 
 * @return wifi_status_t Current WiFi status
 */
wifi_status_t wifi_manager_get_status(void);

/**
 * @brief Get WiFi connection information
 * 
 * @param ip_str Buffer to store IP address string (minimum 16 bytes)
 * @param rssi Pointer to store RSSI value
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_manager_get_info(char *ip_str, int8_t *rssi);

/**
 * @brief Disconnect from WiFi and deinitialize
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t wifi_manager_deinit(void);

/**
 * @brief Check if WiFi is connected
 * 
 * @return bool true if connected, false otherwise
 */
bool wifi_manager_is_connected(void);

#endif // _WIFI_MANAGER_H_ 