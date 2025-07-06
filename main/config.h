#ifndef _CONFIG_H_
#define _CONFIG_H_

// Pin definitions
#define PIN_NUM_SCLK           7  // LCD Clock
#define PIN_NUM_MOSI           6  // LCD MOSI
#define PIN_NUM_MISO          -1  // LCD MISO (not used)
#define PIN_NUM_LCD_DC        15  // LCD DC
#define PIN_NUM_LCD_RST       21  // LCD RST
#define PIN_NUM_LCD_CS        14  // LCD CS
#define PIN_NUM_LCD_BL        22  // LCD Backlight

// RGB LED Pin
#define PIN_RGB_LED           8   // RGB LED Control

// LCD Configuration
#define LCD_PIXEL_CLOCK_HZ    (5 * 1000 * 1000)  // Reduced to 5MHz
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define LCD_H_RES             172
#define LCD_V_RES             320

// LED Configuration
#define LED_ON                0   // LED jest aktywny przy stanie niskim (0)
#define LED_OFF               1   // LED jest wyłączony przy stanie wysokim (1)

// WiFi Configuration - Now using Kconfig (menuconfig)
#define WIFI_SSID             CONFIG_WIFI_SSID
#define WIFI_PASSWORD         CONFIG_WIFI_PASSWORD
#define WIFI_MAXIMUM_RETRY    CONFIG_WIFI_MAXIMUM_RETRY
#define WIFI_TIMEOUT_MS       10000               // Connection timeout in milliseconds

// Aircraft Tracker Configuration - Using Kconfig (menuconfig)
#define TRACKER_LATITUDE      atof(CONFIG_TRACKER_LATITUDE)
#define TRACKER_LONGITUDE     atof(CONFIG_TRACKER_LONGITUDE)
#define TRACKER_RADIUS_KM     CONFIG_TRACKER_RADIUS_KM
#define TRACKER_UPDATE_INTERVAL CONFIG_TRACKER_UPDATE_INTERVAL

// OpenSky Network API (HTTPS)
#define OPENSKY_API_URL       "https://opensky-network.org/api/states/all"

// ADSB.FI API (HTTPS - requires SSL)
#define ADSB_API_URL          "https://opendata.adsb.fi/api/v2/lat"
#define MAX_AIRCRAFT_COUNT    20                  // Maximum aircraft to track
#define HTTP_TIMEOUT_MS       10000               // HTTP request timeout

#endif // _CONFIG_H_ 