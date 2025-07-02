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

#endif // _CONFIG_H_ 