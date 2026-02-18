#include "aircraft_tracker.h"
#include "config.h"
#include <ctype.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cJSON.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "mbedtls/base64.h"

static const char *TAG = "aircraft_tracker";

#define RESP_BUF_SIZE 24576
#define TOKEN_RESP_BUF_SIZE 4096
#define FLIGHTS_RESP_BUF_SIZE 8192
#define ADSBDB_RESP_BUF_SIZE 8192
#define ACCESS_TOKEN_MAX_LEN 2048
#define ENRICHMENT_CACHE_TTL_SEC 300
#define DEG_TO_RAD (0.01745329251994329576923690768489)

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} http_resp_buf_t;

typedef struct {
    bool valid;
    char icao24[16];
    char callsign[16];
    aircraft_info_t data;
} enrichment_cache_t;

static enrichment_cache_t s_enrichment_cache = {0};

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    http_resp_buf_t *resp = (http_resp_buf_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data && evt->data_len > 0) {
        size_t available = (resp->cap > resp->len) ? (resp->cap - resp->len - 1) : 0;
        size_t to_copy = (evt->data_len < available) ? evt->data_len : available;
        if (to_copy > 0) {
            memcpy(resp->buf + resp->len, evt->data, to_copy);
            resp->len += to_copy;
            resp->buf[resp->len] = '\0';
        }
    }
    return ESP_OK;
}

static double deg2rad(double deg)
{
    return deg * DEG_TO_RAD;
}

static float haversine_km(double lat1, double lon1, double lat2, double lon2)
{
    const double earth_radius_km = 6371.0;
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
               cos(deg2rad(lat1)) * cos(deg2rad(lat2)) *
               sin(dlon / 2.0) * sin(dlon / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return (float)(earth_radius_km * c);
}

static double tracker_home_lat(void)
{
    return strtod(TRACKER_HOME_LAT, NULL);
}

static double tracker_home_lon(void)
{
    return strtod(TRACKER_HOME_LON, NULL);
}

static void trim_spaces(char *str)
{
    size_t len;
    char *start = str;
    char *end;

    while (*start && isspace((unsigned char)*start)) {
        start++;
    }
    if (start != str) {
        memmove(str, start, strlen(start) + 1);
    }

    len = strlen(str);
    if (len == 0) {
        return;
    }
    end = str + len - 1;
    while (end >= str && isspace((unsigned char)*end)) {
        *end = '\0';
        end--;
    }
}

static void copy_json_string(cJSON *obj, const char *key, char *out, size_t out_size)
{
    cJSON *item;
    if (obj == NULL || key == NULL || out == NULL || out_size == 0) {
        return;
    }
    item = cJSON_GetObjectItem(obj, key);
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        snprintf(out, out_size, "%s", item->valuestring);
    }
}

static esp_err_t http_get_oauth_token(char *token_out, size_t token_out_size)
{
    const size_t post_body_size = 1024;
    char *response = NULL;
    char *post_body = NULL;
    http_resp_buf_t resp = {
        .buf = NULL,
        .len = 0,
        .cap = 0,
    };

    response = calloc(1, TOKEN_RESP_BUF_SIZE);
    post_body = calloc(1, post_body_size);
    if (response == NULL || post_body == NULL) {
        free(response);
        free(post_body);
        return ESP_ERR_NO_MEM;
    }
    resp.buf = response;
    resp.cap = TOKEN_RESP_BUF_SIZE;

    snprintf(post_body, post_body_size,
             "grant_type=client_credentials&client_id=%s&client_secret=%s",
             TRACKER_OPENSKY_CLIENT_ID, TRACKER_OPENSKY_CLIENT_SECRET);

    esp_http_client_config_t cfg = {
        .url = TRACKER_OPENSKY_TOKEN_URL,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 12000,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        free(response);
        free(post_body);
        return ESP_FAIL;
    }

    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");
    esp_http_client_set_post_field(client, post_body, strlen(post_body));

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OAuth token HTTP failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        free(response);
        free(post_body);
        return err;
    }

    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (status != 200) {
        ESP_LOGW(TAG, "OAuth token status: %d", status);
        free(response);
        free(post_body);
        return ESP_FAIL;
    }

    cJSON *root = cJSON_Parse(response);
    free(post_body);
    post_body = NULL;
    if (root == NULL) {
        free(response);
        return ESP_ERR_INVALID_RESPONSE;
    }

    cJSON *token = cJSON_GetObjectItem(root, "access_token");
    if (!cJSON_IsString(token) || token->valuestring == NULL) {
        cJSON_Delete(root);
        free(response);
        return ESP_ERR_INVALID_RESPONSE;
    }

    snprintf(token_out, token_out_size, "%s", token->valuestring);
    cJSON_Delete(root);
    free(response);
    return ESP_OK;
}

static void set_auth_header_if_available(esp_http_client_handle_t client)
{
    if (strlen(TRACKER_OPENSKY_CLIENT_ID) > 0 && strlen(TRACKER_OPENSKY_CLIENT_SECRET) > 0) {
        char access_token[ACCESS_TOKEN_MAX_LEN] = {0};
        char auth_header[ACCESS_TOKEN_MAX_LEN + 16];
        if (http_get_oauth_token(access_token, sizeof(access_token)) == ESP_OK) {
            snprintf(auth_header, sizeof(auth_header), "Bearer %s", access_token);
            esp_http_client_set_header(client, "Authorization", auth_header);
        }
    } else if (strlen(TRACKER_OPENSKY_USERNAME) > 0) {
        char userpass[160];
        unsigned char b64[240];
        size_t b64_len = 0;
        char auth_header[280];

        snprintf(userpass, sizeof(userpass), "%s:%s", TRACKER_OPENSKY_USERNAME, TRACKER_OPENSKY_PASSWORD);
        if (mbedtls_base64_encode(b64, sizeof(b64), &b64_len, (const unsigned char *)userpass, strlen(userpass)) == 0) {
            snprintf(auth_header, sizeof(auth_header), "Basic %.*s", (int)b64_len, b64);
            esp_http_client_set_header(client, "Authorization", auth_header);
        }
    }
}

static esp_err_t http_get_states(char *out_json, size_t out_size)
{
    double lat = tracker_home_lat();
    double lon = tracker_home_lon();
    double radius_km = (double)TRACKER_RADIUS_KM;
    double lat_delta = radius_km / 111.32;
    double lon_delta = radius_km / (111.32 * cos(deg2rad(lat)));
    char url[320];
    http_resp_buf_t resp = {
        .buf = out_json,
        .len = 0,
        .cap = out_size,
    };

    snprintf(url, sizeof(url),
             "https://opensky-network.org/api/states/all?lamin=%.6f&lomin=%.6f&lamax=%.6f&lomax=%.6f",
             lat - lat_delta, lon - lon_delta, lat + lat_delta, lon + lon_delta);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 12000,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        return ESP_FAIL;
    }

    set_auth_header_if_available(client);

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HTTP request failed: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (status != 200) {
        ESP_LOGW(TAG, "OpenSky HTTP status: %d", status);
        return ESP_FAIL;
    }

    if (resp.len == 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

static esp_err_t http_get_json(const char *url, char *out_json, size_t out_size)
{
    http_resp_buf_t resp = {
        .buf = out_json,
        .len = 0,
        .cap = out_size,
    };

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 12000,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        return ESP_FAIL;
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        esp_http_client_cleanup(client);
        return err;
    }

    if (esp_http_client_get_status_code(client) != 200) {
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    esp_http_client_cleanup(client);
    return (resp.len > 0) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t http_get_flights_for_aircraft(const char *icao24, int64_t begin_unix, int64_t end_unix, char *out_json, size_t out_size)
{
    char url[256];
    http_resp_buf_t resp = {
        .buf = out_json,
        .len = 0,
        .cap = out_size,
    };

    snprintf(url, sizeof(url),
             "https://opensky-network.org/api/flights/aircraft?icao24=%s&begin=%" PRId64 "&end=%" PRId64,
             icao24, begin_unix, end_unix);

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 12000,
        .event_handler = http_event_handler,
        .user_data = &resp,
        .buffer_size = 4096,
        .buffer_size_tx = 2048,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };

    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client == NULL) {
        return ESP_FAIL;
    }
    set_auth_header_if_available(client);

    esp_err_t err = esp_http_client_perform(client);
    if (err != ESP_OK) {
        esp_http_client_cleanup(client);
        return err;
    }

    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    if (status != 200) {
        return ESP_FAIL;
    }

    return (resp.len > 0) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static bool airport_or_empty(cJSON *item, char *out, size_t out_size)
{
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        snprintf(out, out_size, "%s", item->valuestring);
        return out[0] != '\0';
    }
    out[0] = '\0';
    return false;
}

static void fill_route_from_recent_flights(const char *icao24, const char *callsign, int64_t server_time_unix, aircraft_info_t *info)
{
    char *response = NULL;
    cJSON *root = NULL;
    bool prefer_callsign = false;
    int64_t best_last_seen = -1;
    char best_departure[8] = {0};
    char best_arrival[8] = {0};
    size_t i;

    if (icao24 == NULL || icao24[0] == '\0' || server_time_unix <= 0 || info == NULL) {
        return;
    }

    response = calloc(1, FLIGHTS_RESP_BUF_SIZE);
    if (response == NULL) {
        return;
    }

    if (http_get_flights_for_aircraft(icao24, server_time_unix - 21600, server_time_unix + 300, response, FLIGHTS_RESP_BUF_SIZE) != ESP_OK) {
        free(response);
        return;
    }

    root = cJSON_Parse(response);
    free(response);
    response = NULL;
    if (!cJSON_IsArray(root)) {
        cJSON_Delete(root);
        return;
    }

    for (i = 0; i < (size_t)cJSON_GetArraySize(root); i++) {
        cJSON *flight = cJSON_GetArrayItem(root, (int)i);
        cJSON *dep = cJSON_GetObjectItem(flight, "estDepartureAirport");
        cJSON *arr = cJSON_GetObjectItem(flight, "estArrivalAirport");
        cJSON *last_seen_item = cJSON_GetObjectItem(flight, "lastSeen");
        cJSON *flight_callsign_item = cJSON_GetObjectItem(flight, "callsign");
        bool has_dep;
        bool has_arr;
        bool callsign_match = false;
        int64_t last_seen = 0;
        char dep_code[8] = {0};
        char arr_code[8] = {0};

        if (!cJSON_IsObject(flight)) {
            continue;
        }

        has_dep = airport_or_empty(dep, dep_code, sizeof(dep_code));
        has_arr = airport_or_empty(arr, arr_code, sizeof(arr_code));
        if (!has_dep && !has_arr) {
            continue;
        }

        if (cJSON_IsNumber(last_seen_item)) {
            last_seen = (int64_t)last_seen_item->valuedouble;
        }

        if (cJSON_IsString(flight_callsign_item) && flight_callsign_item->valuestring && callsign && callsign[0]) {
            char tmp_callsign[16];
            snprintf(tmp_callsign, sizeof(tmp_callsign), "%s", flight_callsign_item->valuestring);
            trim_spaces(tmp_callsign);
            callsign_match = (strcmp(tmp_callsign, callsign) == 0);
        }

        if (callsign_match && !prefer_callsign) {
            prefer_callsign = true;
            best_last_seen = -1;
        }
        if (prefer_callsign && !callsign_match) {
            continue;
        }

        if (last_seen >= best_last_seen) {
            best_last_seen = last_seen;
            snprintf(best_departure, sizeof(best_departure), "%s", has_dep ? dep_code : "");
            snprintf(best_arrival, sizeof(best_arrival), "%s", has_arr ? arr_code : "");
        }
    }

    cJSON_Delete(root);

    if (best_last_seen >= 0) {
        snprintf(info->origin_airport, sizeof(info->origin_airport), "%s", best_departure);
        snprintf(info->destination_airport, sizeof(info->destination_airport), "%s", best_arrival);
    }
}

static void fill_from_adsbdb(const char *icao24, const char *callsign, aircraft_info_t *info)
{
    char *response = NULL;
    cJSON *root = NULL;
    cJSON *resp_obj = NULL;
    char url[256];

    if (icao24 == NULL || icao24[0] == '\0' || info == NULL) {
        return;
    }

    if (s_enrichment_cache.valid &&
        strcmp(s_enrichment_cache.icao24, icao24) == 0 &&
        strcmp(s_enrichment_cache.callsign, (callsign != NULL) ? callsign : "") == 0) {
        if (s_enrichment_cache.data.registration[0] != '\0') {
            snprintf(info->registration, sizeof(info->registration), "%s", s_enrichment_cache.data.registration);
        }
        if (s_enrichment_cache.data.aircraft_type_code[0] != '\0') {
            snprintf(info->aircraft_type_code, sizeof(info->aircraft_type_code), "%s", s_enrichment_cache.data.aircraft_type_code);
        }
        if (s_enrichment_cache.data.aircraft_type_name[0] != '\0') {
            snprintf(info->aircraft_type_name, sizeof(info->aircraft_type_name), "%s", s_enrichment_cache.data.aircraft_type_name);
        }
        if (s_enrichment_cache.data.origin_airport[0] != '\0') {
            snprintf(info->origin_airport, sizeof(info->origin_airport), "%s", s_enrichment_cache.data.origin_airport);
        }
        if (s_enrichment_cache.data.destination_airport[0] != '\0') {
            snprintf(info->destination_airport, sizeof(info->destination_airport), "%s", s_enrichment_cache.data.destination_airport);
        }
        return;
    }

    response = calloc(1, ADSBDB_RESP_BUF_SIZE);
    if (response == NULL) {
        return;
    }

    if (callsign != NULL && callsign[0] != '\0' && strcmp(callsign, "N/A") != 0) {
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/aircraft/%s?callsign=%s", icao24, callsign);
    } else {
        snprintf(url, sizeof(url), "https://api.adsbdb.com/v0/aircraft/%s", icao24);
    }

    if (http_get_json(url, response, ADSBDB_RESP_BUF_SIZE) != ESP_OK) {
        free(response);
        return;
    }

    root = cJSON_Parse(response);
    free(response);
    response = NULL;
    if (!cJSON_IsObject(root)) {
        cJSON_Delete(root);
        return;
    }

    resp_obj = cJSON_GetObjectItem(root, "response");
    if (cJSON_IsObject(resp_obj)) {
        cJSON *aircraft = cJSON_GetObjectItem(resp_obj, "aircraft");
        cJSON *flightroute = cJSON_GetObjectItem(resp_obj, "flightroute");

        if (cJSON_IsObject(aircraft)) {
            copy_json_string(aircraft, "registration", info->registration, sizeof(info->registration));
            copy_json_string(aircraft, "icao_type", info->aircraft_type_code, sizeof(info->aircraft_type_code));
            copy_json_string(aircraft, "type", info->aircraft_type_name, sizeof(info->aircraft_type_name));
        }

        if (cJSON_IsObject(flightroute)) {
            cJSON *origin = cJSON_GetObjectItem(flightroute, "origin");
            cJSON *destination = cJSON_GetObjectItem(flightroute, "destination");

            if (cJSON_IsObject(origin)) {
                copy_json_string(origin, "iata_code", info->origin_airport, sizeof(info->origin_airport));
                if (info->origin_airport[0] == '\0') {
                    copy_json_string(origin, "icao_code", info->origin_airport, sizeof(info->origin_airport));
                }
            }
            if (cJSON_IsObject(destination)) {
                copy_json_string(destination, "iata_code", info->destination_airport, sizeof(info->destination_airport));
                if (info->destination_airport[0] == '\0') {
                    copy_json_string(destination, "icao_code", info->destination_airport, sizeof(info->destination_airport));
                }
            }
        }
    }

    cJSON_Delete(root);

    memset(&s_enrichment_cache, 0, sizeof(s_enrichment_cache));
    s_enrichment_cache.valid = true;
    snprintf(s_enrichment_cache.icao24, sizeof(s_enrichment_cache.icao24), "%s", icao24);
    snprintf(s_enrichment_cache.callsign, sizeof(s_enrichment_cache.callsign), "%s", (callsign != NULL) ? callsign : "");
    s_enrichment_cache.data = *info;
}

esp_err_t aircraft_tracker_fetch_nearest(aircraft_info_t *out_info)
{
    char *response = NULL;
    double home_lat = tracker_home_lat();
    double home_lon = tracker_home_lon();
    cJSON *root = NULL;
    cJSON *states = NULL;
    size_t i;
    float best_dist = 1000000.0f;
    bool found = false;
    aircraft_info_t best = {0};
    char best_icao24[16] = {0};
    int64_t states_time_unix = 0;

    if (out_info == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_info, 0, sizeof(*out_info));
    response = calloc(1, RESP_BUF_SIZE);
    if (response == NULL) {
        return ESP_ERR_NO_MEM;
    }

    esp_err_t err = http_get_states(response, RESP_BUF_SIZE);
    if (err != ESP_OK) {
        free(response);
        return err;
    }

    root = cJSON_Parse(response);
    free(response);
    response = NULL;
    if (root == NULL) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    states = cJSON_GetObjectItem(root, "states");
    {
        cJSON *time_item = cJSON_GetObjectItem(root, "time");
        if (cJSON_IsNumber(time_item)) {
            states_time_unix = (int64_t)time_item->valuedouble;
        }
    }
    if (!cJSON_IsArray(states)) {
        cJSON_Delete(root);
        return ESP_ERR_NOT_FOUND;
    }

    for (i = 0; i < (size_t)cJSON_GetArraySize(states); i++) {
        cJSON *state = cJSON_GetArrayItem(states, (int)i);
        cJSON *icao24_item;
        cJSON *callsign_item;
        cJSON *lon_item;
        cJSON *lat_item;
        cJSON *baro_alt_item;
        cJSON *geo_alt_item;
        cJSON *track_item;
        cJSON *on_ground_item;
        double lon;
        double lat;
        float dist;
        bool on_ground = false;
        int heading = -1;
        int altitude = -1;
        char callsign[16] = "N/A";
        char icao24[16] = {0};

        if (!cJSON_IsArray(state) || cJSON_GetArraySize(state) < 14) {
            continue;
        }

        icao24_item = cJSON_GetArrayItem(state, 0);
        callsign_item = cJSON_GetArrayItem(state, 1);
        lon_item = cJSON_GetArrayItem(state, 5);
        lat_item = cJSON_GetArrayItem(state, 6);
        baro_alt_item = cJSON_GetArrayItem(state, 7);
        on_ground_item = cJSON_GetArrayItem(state, 8);
        track_item = cJSON_GetArrayItem(state, 10);
        geo_alt_item = cJSON_GetArrayItem(state, 13);

        if (cJSON_IsBool(on_ground_item)) {
            on_ground = cJSON_IsTrue(on_ground_item);
        }
        if (on_ground) {
            continue;
        }

        if (!cJSON_IsNumber(lat_item) || !cJSON_IsNumber(lon_item)) {
            continue;
        }
        lat = lat_item->valuedouble;
        lon = lon_item->valuedouble;

        if (cJSON_IsNumber(track_item)) {
            heading = (int)lround(track_item->valuedouble);
        }

        if (cJSON_IsNumber(geo_alt_item)) {
            altitude = (int)lround(geo_alt_item->valuedouble);
        } else if (cJSON_IsNumber(baro_alt_item)) {
            altitude = (int)lround(baro_alt_item->valuedouble);
        }

        if (cJSON_IsString(callsign_item) && callsign_item->valuestring != NULL) {
            snprintf(callsign, sizeof(callsign), "%s", callsign_item->valuestring);
            trim_spaces(callsign);
            if (callsign[0] == '\0') {
                snprintf(callsign, sizeof(callsign), "N/A");
            }
        }
        if (cJSON_IsString(icao24_item) && icao24_item->valuestring != NULL) {
            snprintf(icao24, sizeof(icao24), "%s", icao24_item->valuestring);
            trim_spaces(icao24);
        }

        dist = haversine_km(home_lat, home_lon, lat, lon);
        if (dist < best_dist) {
            best_dist = dist;
            memset(&best, 0, sizeof(best));
            best.valid = true;
            best.distance_km = dist;
            best.altitude_m = altitude;
            best.heading_deg = heading;
            snprintf(best.callsign, sizeof(best.callsign), "%s", callsign);
            snprintf(best_icao24, sizeof(best_icao24), "%s", icao24);
            found = true;
        }
    }

    cJSON_Delete(root);

    if (!found) {
        return ESP_ERR_NOT_FOUND;
    }

    fill_from_adsbdb(best_icao24, best.callsign, &best);
    if (best.origin_airport[0] == '\0' && best.destination_airport[0] == '\0') {
        fill_route_from_recent_flights(best_icao24, best.callsign, states_time_unix, &best);
    }

    *out_info = best;
    return ESP_OK;
}
