#include "aircraft_tracker.h"
#include "config.h"
#include "wifi_manager.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "cJSON.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>
#include <time.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

static const char *TAG = "aircraft_tracker";

// Global state
static aircraft_data_t g_aircraft_list[MAX_AIRCRAFT_COUNT];
static tracker_stats_t g_tracker_stats = {0};
static TaskHandle_t g_tracker_task_handle = NULL;
static SemaphoreHandle_t g_aircraft_mutex = NULL;
static bool g_tracker_running = false;

// HTTP response buffer - reduced for memory constraints
#define HTTP_BUFFER_SIZE 4096
static char g_http_buffer[HTTP_BUFFER_SIZE];
static int g_http_buffer_len = 0;

// Function prototypes
static void aircraft_tracker_task(void *pvParameters);
static esp_err_t fetch_aircraft_data(void);
static esp_err_t parse_opensky_response(const char *json_data);
static float calculate_distance(float lat1, float lon1, float lat2, float lon2);
static uint8_t calculate_direction(float lat1, float lon1, float lat2, float lon2);
static esp_err_t http_event_handler(esp_http_client_event_t *evt);

esp_err_t aircraft_tracker_init(void)
{
    ESP_LOGI(TAG, "Initializing aircraft tracker");
    
    // Create mutex for thread safety
    g_aircraft_mutex = xSemaphoreCreateMutex();
    if (g_aircraft_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create aircraft mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize aircraft list
    memset(g_aircraft_list, 0, sizeof(g_aircraft_list));
    
    // Initialize stats
    g_tracker_stats.status = TRACKER_STATUS_DISABLED;
    g_tracker_stats.total_requests = 0;
    g_tracker_stats.successful_requests = 0;
    g_tracker_stats.last_update_time = 0;
    g_tracker_stats.aircraft_count = 0;
    
    ESP_LOGI(TAG, "Aircraft tracker initialized successfully");
    ESP_LOGI(TAG, "Search area: %.4f,%.4f radius %.1f km", 
             TRACKER_LATITUDE, TRACKER_LONGITUDE, (float)TRACKER_RADIUS_KM);
    
    return ESP_OK;
}

esp_err_t aircraft_tracker_start(void)
{
    if (g_tracker_running) {
        ESP_LOGW(TAG, "Aircraft tracker already running");
        return ESP_OK;
    }
    
    // Check WiFi connection
    if (!wifi_manager_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected, cannot start tracker");
        g_tracker_stats.status = TRACKER_STATUS_NO_WIFI;
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "WiFi is connected, proceeding with tracker start");
    
    g_tracker_stats.status = TRACKER_STATUS_INITIALIZING;
    
    // Create tracker task with optimized stack and lower priority
    ESP_LOGI(TAG, "Creating aircraft tracker task...");
    ESP_LOGI(TAG, "Free heap before task creation: %u bytes", esp_get_free_heap_size());
    BaseType_t ret = xTaskCreate(
        aircraft_tracker_task,
        "aircraft_tracker",
        8192,   // Optimized stack size (8KB)
        NULL,
        3,      // Lower priority (less conflicts)
        &g_tracker_task_handle
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create aircraft tracker task, ret=%d", ret);
        ESP_LOGE(TAG, "Free heap: %u bytes", esp_get_free_heap_size());
        g_tracker_stats.status = TRACKER_STATUS_ERROR;
        return ESP_ERR_NO_MEM;
    }
    
    ESP_LOGI(TAG, "Aircraft tracker task created successfully");
    
    g_tracker_running = true;
    g_tracker_stats.status = TRACKER_STATUS_RUNNING;
    
    ESP_LOGI(TAG, "Aircraft tracker started successfully");
    return ESP_OK;
}

esp_err_t aircraft_tracker_stop(void)
{
    if (!g_tracker_running) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Stopping aircraft tracker");
    
    g_tracker_running = false;
    
    if (g_tracker_task_handle != NULL) {
        vTaskDelete(g_tracker_task_handle);
        g_tracker_task_handle = NULL;
    }
    
    g_tracker_stats.status = TRACKER_STATUS_DISABLED;
    
    ESP_LOGI(TAG, "Aircraft tracker stopped");
    return ESP_OK;
}

esp_err_t aircraft_tracker_get_aircraft(aircraft_data_t *aircraft_list, uint8_t max_count, uint8_t *actual_count)
{
    if (aircraft_list == NULL || actual_count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *actual_count = 0;
    
    if (xSemaphoreTake(g_aircraft_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        uint8_t count = 0;
        for (int i = 0; i < MAX_AIRCRAFT_COUNT && count < max_count; i++) {
            if (g_aircraft_list[i].valid) {
                aircraft_list[count] = g_aircraft_list[i];
                count++;
            }
        }
        *actual_count = count;
        xSemaphoreGive(g_aircraft_mutex);
        return ESP_OK;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t aircraft_tracker_get_stats(tracker_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    *stats = g_tracker_stats;
    return ESP_OK;
}

esp_err_t aircraft_tracker_force_update(void)
{
    if (!g_tracker_running) {
        return ESP_ERR_INVALID_STATE;
    }
    
    return fetch_aircraft_data();
}

bool aircraft_tracker_is_running(void)
{
    return g_tracker_running;
}

static void aircraft_tracker_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Aircraft tracker task started");
    ESP_LOGI(TAG, "Update interval: %d seconds", TRACKER_UPDATE_INTERVAL);
    ESP_LOGI(TAG, "Initial g_tracker_running = %d", g_tracker_running);
    ESP_LOGI(TAG, "Task stack high water mark: %u bytes", uxTaskGetStackHighWaterMark(NULL));
    ESP_LOGI(TAG, "Free heap at task start: %u bytes", esp_get_free_heap_size());
    
    // Larger delay to ensure system stability
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2 seconds
    
    ESP_LOGI(TAG, "About to enter while loop, g_tracker_running = %d", g_tracker_running);
    
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t update_frequency = pdMS_TO_TICKS(TRACKER_UPDATE_INTERVAL * 1000);
    
    while (g_tracker_running) {
        // Check WiFi connection
        bool wifi_connected = wifi_manager_is_connected();
        wifi_status_t wifi_status = wifi_manager_get_status();
        ESP_LOGI(TAG, "Task loop: wifi_connected=%d, wifi_status=%d, running=%d", 
                 wifi_connected, wifi_status, g_tracker_running);
        
        if (!wifi_connected) {
            ESP_LOGW(TAG, "WiFi disconnected (status=%d), waiting for reconnection", wifi_status);
            g_tracker_stats.status = TRACKER_STATUS_NO_WIFI;
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5000)); // Check every 5 seconds
            continue;
        }
        
        g_tracker_stats.status = TRACKER_STATUS_RUNNING;
        ESP_LOGI(TAG, "WiFi connected, fetching aircraft data (request #%d)", g_tracker_stats.total_requests + 1);
        
        // Fetch aircraft data
        esp_err_t ret = fetch_aircraft_data();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Aircraft data updated successfully, found %d aircraft", 
                     g_tracker_stats.aircraft_count);
        } else {
            ESP_LOGW(TAG, "Failed to update aircraft data: %s", esp_err_to_name(ret));
        }
        
        // Wait for next update
        vTaskDelayUntil(&last_wake_time, update_frequency);
    }
    
    ESP_LOGI(TAG, "Aircraft tracker task ended");
    vTaskDelete(NULL);
}

static esp_err_t test_internet_connectivity(void)
{
    ESP_LOGI(TAG, "Testing internet connectivity...");
    
    // Very simple HTTP test to a reliable server
    esp_http_client_config_t config = {
        .url = "http://neverssl.com/",  // Simple HTTP site without redirects
        .method = HTTP_METHOD_GET,
        .timeout_ms = 15000,
        .skip_cert_common_name_check = true,
        .transport_type = HTTP_TRANSPORT_OVER_TCP,  // Force HTTP (no SSL)
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client for connectivity test");
        return ESP_FAIL;
    }
    
    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);
    
    ESP_LOGI(TAG, "Connectivity test result: err=%s, status=%d, length=%d", 
             esp_err_to_name(err), status_code, content_length);
    
    esp_http_client_cleanup(client);
    
    if (err == ESP_OK && status_code == 200) {
        ESP_LOGI(TAG, "✅ Internet connectivity OK");
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "❌ Internet connectivity FAILED");
        return ESP_FAIL;
    }
}

static esp_err_t fetch_aircraft_data(void)
{
    ESP_LOGI(TAG, "fetch_aircraft_data() called");
    ESP_LOGI(TAG, "Free heap before HTTP request: %u bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Minimum free heap: %u bytes", esp_get_minimum_free_heap_size());
    
    // Test internet connectivity first (only on first few requests)
    static uint8_t connectivity_test_count = 0;
    if (connectivity_test_count < 3) {
        // Test DNS resolution first
        struct addrinfo hints = {0};
        struct addrinfo *result = NULL;
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        
        int dns_result = getaddrinfo("google.com", "80", &hints, &result);
        if (dns_result == 0 && result != NULL) {
            struct sockaddr_in* addr_in = (struct sockaddr_in*)result->ai_addr;
            ESP_LOGI(TAG, "✅ DNS works - google.com resolved to %s", 
                     inet_ntoa(addr_in->sin_addr));
            freeaddrinfo(result);
        } else {
            ESP_LOGW(TAG, "❌ DNS failed - cannot resolve google.com, error: %d", dns_result);
        }
        
        test_internet_connectivity();
        connectivity_test_count++;
    }
    
    // Build OpenSky Network API URL with bounding box
    // Calculate bounding box around our location
    float lat_min = TRACKER_LATITUDE - 0.1f;
    float lat_max = TRACKER_LATITUDE + 0.1f;
    float lon_min = TRACKER_LONGITUDE - 0.1f;
    float lon_max = TRACKER_LONGITUDE + 0.1f;
    
    ESP_LOGI(TAG, "DEBUG: Using OpenSky Network API");
    ESP_LOGI(TAG, "DEBUG: Bounding box: lat[%.4f,%.4f] lon[%.4f,%.4f]", 
             lat_min, lat_max, lon_min, lon_max);
    
    char url[256];
    snprintf(url, sizeof(url), 
             "%s?bbox=%.4f,%.4f,%.4f,%.4f",
             OPENSKY_API_URL, lat_min, lat_max, lon_min, lon_max);
    
    ESP_LOGI(TAG, "Fetching data from: %s", url);
        ESP_LOGI(TAG, "Coordinates: lat=%.4f, lon=%.4f, radius=%d km",
             TRACKER_LATITUDE, TRACKER_LONGITUDE, TRACKER_RADIUS_KM);
    
    // Reset HTTP buffer
    g_http_buffer_len = 0;
    memset(g_http_buffer, 0, HTTP_BUFFER_SIZE);
    
    // HTTPS configuration with minimal SSL requirements  
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = HTTP_TIMEOUT_MS,
        .event_handler = http_event_handler,
        .buffer_size = 4096,                        // Zwiększony buffer
        .buffer_size_tx = 2048,                     // Zwiększony TX buffer
        .transport_type = HTTP_TRANSPORT_OVER_SSL,  // Use HTTPS
        .skip_cert_common_name_check = true,        // Skip SSL certificate verification
        .use_global_ca_store = false,               // Don't use global CA store
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return ESP_FAIL;
    }
    
    g_tracker_stats.total_requests++;
    
    // Perform HTTP request
    esp_err_t err = esp_http_client_perform(client);
    
    if (err == ESP_OK) {
        int status_code = esp_http_client_get_status_code(client);
        if (status_code == 200) {
            ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %d", 
                     status_code, g_http_buffer_len);
            
            // Show response content for debugging
            ESP_LOGI(TAG, "=== HTTP RESPONSE START ===");
            ESP_LOGI(TAG, "%.*s", (int)g_http_buffer_len, g_http_buffer);
            ESP_LOGI(TAG, "=== HTTP RESPONSE END ===");
            
            // Parse JSON response
            err = parse_opensky_response(g_http_buffer);
            if (err == ESP_OK) {
                g_tracker_stats.successful_requests++;
                g_tracker_stats.last_update_time = (uint32_t)time(NULL);
            }
        } else {
            ESP_LOGW(TAG, "HTTP GET failed with status %d", status_code);
            err = ESP_FAIL;
        }
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    return err;
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (g_http_buffer_len + evt->data_len < HTTP_BUFFER_SIZE) {
                memcpy(g_http_buffer + g_http_buffer_len, evt->data, evt->data_len);
                g_http_buffer_len += evt->data_len;
                g_http_buffer[g_http_buffer_len] = '\0';
            } else {
                ESP_LOGW(TAG, "HTTP response too large, truncating");
            }
            break;
        default:
            break;
    }
    return ESP_OK;
}

static esp_err_t parse_opensky_response(const char *json_data)
{
    cJSON *json = cJSON_Parse(json_data);
    if (json == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON response");
        return ESP_FAIL;
    }
    
    cJSON *states_array = cJSON_GetObjectItem(json, "states");
    if (!cJSON_IsArray(states_array)) {
        ESP_LOGW(TAG, "No 'states' array in response");
        cJSON_Delete(json);
        return ESP_FAIL;
    }
    
    if (xSemaphoreTake(g_aircraft_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to take aircraft mutex");
        cJSON_Delete(json);
        return ESP_ERR_TIMEOUT;
    }
    
    // Clear existing aircraft data
    memset(g_aircraft_list, 0, sizeof(g_aircraft_list));
    
    int aircraft_count = 0;
    int array_size = cJSON_GetArraySize(states_array);
    
    for (int i = 0; i < array_size && aircraft_count < MAX_AIRCRAFT_COUNT; i++) {
        cJSON *state = cJSON_GetArrayItem(states_array, i);
        if (!cJSON_IsArray(state)) {
            continue;
        }
        
        aircraft_data_t *aircraft = &g_aircraft_list[aircraft_count];
        
        // Parse OpenSky Network format (array format)
        // [0]: icao24 (string)
        cJSON *icao24 = cJSON_GetArrayItem(state, 0);
        // [1]: callsign (string)
        cJSON *callsign = cJSON_GetArrayItem(state, 1);
        // [5]: longitude, [6]: latitude (numbers)
        cJSON *longitude = cJSON_GetArrayItem(state, 5);
        cJSON *latitude = cJSON_GetArrayItem(state, 6);
        // [7]: baro_altitude (meters)
        cJSON *altitude = cJSON_GetArrayItem(state, 7);
        // [9]: velocity (m/s)
        cJSON *velocity = cJSON_GetArrayItem(state, 9);
        // [10]: true_track (degrees)
        cJSON *track = cJSON_GetArrayItem(state, 10);
        
        // Validate required fields
        if (!cJSON_IsString(icao24) || !cJSON_IsNumber(latitude) || !cJSON_IsNumber(longitude)) {
            continue;
        }
        
        // Copy ICAO24
        strncpy(aircraft->icao24, cJSON_GetStringValue(icao24), sizeof(aircraft->icao24) - 1);
        aircraft->icao24[sizeof(aircraft->icao24) - 1] = '\0';
        
        // Copy callsign (may be null)
        if (cJSON_IsString(callsign)) {
            const char *cs = cJSON_GetStringValue(callsign);
            if (cs != NULL && strlen(cs) > 0) {
                strncpy(aircraft->callsign, cs, sizeof(aircraft->callsign) - 1);
                aircraft->callsign[sizeof(aircraft->callsign) - 1] = '\0';
                // Remove trailing spaces
                for (int j = strlen(aircraft->callsign) - 1; j >= 0 && aircraft->callsign[j] == ' '; j--) {
                    aircraft->callsign[j] = '\0';
                }
            }
        }
        
        // Parse numeric fields
        aircraft->latitude = (float)cJSON_GetNumberValue(latitude);
        aircraft->longitude = (float)cJSON_GetNumberValue(longitude);
        
        // OpenSky altitude is in meters, convert to feet
        if (cJSON_IsNumber(altitude)) {
            float alt_meters = (float)cJSON_GetNumberValue(altitude);
            aircraft->altitude = alt_meters * 3.28084f; // meters to feet
        }
        
        // OpenSky velocity is in m/s, convert to knots
        if (cJSON_IsNumber(velocity)) {
            float vel_ms = (float)cJSON_GetNumberValue(velocity);
            aircraft->velocity = vel_ms * 1.94384f; // m/s to knots
        }
        
        if (cJSON_IsNumber(track)) {
            aircraft->heading = (float)cJSON_GetNumberValue(track);
        }
        
        // Set last contact to current time (OpenSky doesn't provide this directly)
        aircraft->last_contact = (uint32_t)time(NULL);
        
        // Calculate distance and direction from our position
        aircraft->distance_km = calculate_distance(
            TRACKER_LATITUDE, TRACKER_LONGITUDE,
            aircraft->latitude, aircraft->longitude
        );
        
        aircraft->direction = calculate_direction(
            TRACKER_LATITUDE, TRACKER_LONGITUDE,
            aircraft->latitude, aircraft->longitude
        );
        
        aircraft->valid = true;
        aircraft_count++;
    }
    
    g_tracker_stats.aircraft_count = aircraft_count;
    
    xSemaphoreGive(g_aircraft_mutex);
    cJSON_Delete(json);
    
    ESP_LOGI(TAG, "Parsed %d aircraft from OpenSky response", aircraft_count);
    return ESP_OK;
}

static float calculate_distance(float lat1, float lon1, float lat2, float lon2)
{
    // Haversine formula
    const float R = 6371.0f; // Earth radius in km
    
    float dLat = (lat2 - lat1) * M_PI / 180.0f;
    float dLon = (lon2 - lon1) * M_PI / 180.0f;
    
    float a = sinf(dLat/2) * sinf(dLat/2) +
              cosf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) *
              sinf(dLon/2) * sinf(dLon/2);
    
    float c = 2 * atan2f(sqrtf(a), sqrtf(1-a));
    
    return R * c;
}

static uint8_t calculate_direction(float lat1, float lon1, float lat2, float lon2)
{
    // Calculate bearing and convert to 8-direction compass
    float dLon = (lon2 - lon1) * M_PI / 180.0f;
    float y = sinf(dLon) * cosf(lat2 * M_PI / 180.0f);
    float x = cosf(lat1 * M_PI / 180.0f) * sinf(lat2 * M_PI / 180.0f) -
              sinf(lat1 * M_PI / 180.0f) * cosf(lat2 * M_PI / 180.0f) * cosf(dLon);
    
    float bearing = atan2f(y, x) * 180.0f / M_PI;
    if (bearing < 0) bearing += 360.0f;
    
    // Convert to 8 directions (0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW)
    return (uint8_t)((bearing + 22.5f) / 45.0f) % 8;
} 