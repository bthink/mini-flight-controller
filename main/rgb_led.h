#ifndef _RGB_LED_H_
#define _RGB_LED_H_

#include <stdint.h>
#include "esp_err.h"

// RGB LED Colors
typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} rgb_color_t;

// Predefined colors
#define RGB_RED     {255, 0, 0}
#define RGB_GREEN   {0, 255, 0}
#define RGB_BLUE    {0, 0, 255}
#define RGB_YELLOW  {255, 255, 0}
#define RGB_CYAN    {0, 255, 255}
#define RGB_MAGENTA {255, 0, 255}
#define RGB_WHITE   {255, 255, 255}
#define RGB_OFF     {0, 0, 0}

// LED modes
typedef enum {
    RGB_MODE_STATIC,
    RGB_MODE_BLINK,
    RGB_MODE_RAINBOW,
    RGB_MODE_BREATHE,
    RGB_MODE_FLIGHT_STATUS
} rgb_led_mode_t;

/**
 * @brief Initialize RGB LED
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_init(void);

/**
 * @brief Set RGB LED color
 * 
 * @param color RGB color to set
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_set_color(rgb_color_t color);

/**
 * @brief Set RGB LED color with individual values
 * 
 * @param red Red component (0-255)
 * @param green Green component (0-255) 
 * @param blue Blue component (0-255)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue);

/**
 * @brief Turn off RGB LED
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_off(void);

/**
 * @brief Set LED mode and start task
 * 
 * @param mode LED animation mode
 * @param period_ms Period in milliseconds for animations
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_set_mode(rgb_led_mode_t mode, uint32_t period_ms);

/**
 * @brief Stop LED animation task
 * 
 * @return esp_err_t ESP_OK on success
 */
esp_err_t rgb_led_stop(void);

#endif // _RGB_LED_H_ 