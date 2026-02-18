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

// Aircraft tracker (OpenSky API) configuration
#define TRACKER_HOME_LAT            CONFIG_TRACKER_HOME_LAT
#define TRACKER_HOME_LON            CONFIG_TRACKER_HOME_LON
#define TRACKER_RADIUS_KM           CONFIG_TRACKER_RADIUS_KM
#define TRACKER_REFRESH_SEC         CONFIG_TRACKER_REFRESH_SEC
#define TRACKER_OPENSKY_USERNAME    CONFIG_TRACKER_OPENSKY_USERNAME
#define TRACKER_OPENSKY_PASSWORD    CONFIG_TRACKER_OPENSKY_PASSWORD

#ifdef CONFIG_TRACKER_OPENSKY_CLIENT_ID
#define TRACKER_OPENSKY_CLIENT_ID   CONFIG_TRACKER_OPENSKY_CLIENT_ID
#else
#define TRACKER_OPENSKY_CLIENT_ID   ""
#endif

#ifdef CONFIG_TRACKER_OPENSKY_CLIENT_SECRET
#define TRACKER_OPENSKY_CLIENT_SECRET CONFIG_TRACKER_OPENSKY_CLIENT_SECRET
#else
#define TRACKER_OPENSKY_CLIENT_SECRET ""
#endif

#ifdef CONFIG_TRACKER_OPENSKY_TOKEN_URL
#define TRACKER_OPENSKY_TOKEN_URL   CONFIG_TRACKER_OPENSKY_TOKEN_URL
#else
#define TRACKER_OPENSKY_TOKEN_URL   "https://auth.opensky-network.org/auth/realms/opensky-network/protocol/openid-connect/token"
#endif

#endif // _CONFIG_H_
