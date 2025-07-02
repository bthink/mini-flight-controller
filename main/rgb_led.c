#include "rgb_led.h"
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "led_strip.h"
#include "esp_log.h"
#include <math.h>

// Define M_PI if not available
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "rgb_led";

static led_strip_handle_t led_strip = NULL;
static TaskHandle_t rgb_task_handle = NULL;
static rgb_led_mode_t current_mode = RGB_MODE_STATIC;
static uint32_t animation_period = 1000;
static SemaphoreHandle_t rgb_mutex = NULL;

// Private function declarations
static void rgb_led_task(void *pvParameters);
static void rainbow_effect(uint32_t step);
static void breathe_effect(uint32_t step, rgb_color_t base_color);

esp_err_t rgb_led_init(void)
{
    ESP_LOGI(TAG, "Initializing RGB LED on pin %d", PIN_RGB_LED);
    
    // Create mutex for thread safety
    rgb_mutex = xSemaphoreCreateMutex();
    if (rgb_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create RGB mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // LED strip configuration
    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_RGB_LED,
        .max_leds = 1, // Single RGB LED
    };
    
    esp_err_t ret = ESP_FAIL;
    
    // Try RMT backend first
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    
    ret = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "RMT backend failed, trying SPI backend");
        
        // Fallback to SPI backend if RMT fails
        led_strip_spi_config_t spi_config = {
            .spi_bus = SPI2_HOST, // Use SPI2 for RGB LED
            .flags.with_dma = true,
        };
        
        ret = led_strip_new_spi_device(&strip_config, &spi_config, &led_strip);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Both RMT and SPI backends failed: %s", esp_err_to_name(ret));
            vSemaphoreDelete(rgb_mutex);
            rgb_mutex = NULL;
            return ret;
        }
        ESP_LOGI(TAG, "Using SPI backend for RGB LED");
    } else {
        ESP_LOGI(TAG, "Using RMT backend for RGB LED");
    }
    
    // Turn off LED initially
    led_strip_clear(led_strip);
    
    ESP_LOGI(TAG, "RGB LED initialized successfully");
    return ESP_OK;
}

esp_err_t rgb_led_set_color(rgb_color_t color)
{
    if (led_strip == NULL) {
        ESP_LOGE(TAG, "LED strip not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(rgb_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_err_t ret = led_strip_set_pixel(led_strip, 0, color.red, color.green, color.blue);
        if (ret == ESP_OK) {
            ret = led_strip_refresh(led_strip);
        }
        xSemaphoreGive(rgb_mutex);
        return ret;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t rgb_led_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    rgb_color_t color = {red, green, blue};
    return rgb_led_set_color(color);
}

esp_err_t rgb_led_off(void)
{
    if (led_strip == NULL) {
        ESP_LOGE(TAG, "LED strip not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTake(rgb_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_err_t ret = led_strip_clear(led_strip);
        xSemaphoreGive(rgb_mutex);
        return ret;
    }
    
    return ESP_ERR_TIMEOUT;
}

esp_err_t rgb_led_set_mode(rgb_led_mode_t mode, uint32_t period_ms)
{
    if (led_strip == NULL) {
        ESP_LOGE(TAG, "LED strip not initialized, cannot set mode");
        return ESP_ERR_INVALID_STATE;
    }
    
    current_mode = mode;
    animation_period = period_ms;
    
    // Stop existing task if running
    if (rgb_task_handle != NULL) {
        vTaskDelete(rgb_task_handle);
        rgb_task_handle = NULL;
    }
    
    // Create new task for animations
    if (mode != RGB_MODE_STATIC) {
        BaseType_t ret = xTaskCreate(rgb_led_task, "rgb_led_task", 4096, NULL, 5, &rgb_task_handle);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "Failed to create RGB LED task");
            return ESP_ERR_NO_MEM;
        }
        ESP_LOGI(TAG, "RGB LED mode set to %d with period %lu ms", mode, period_ms);
    }
    
    return ESP_OK;
}

esp_err_t rgb_led_stop(void)
{
    if (rgb_task_handle != NULL) {
        vTaskDelete(rgb_task_handle);
        rgb_task_handle = NULL;
    }
    
    return rgb_led_off();
}

bool rgb_led_is_initialized(void)
{
    return (led_strip != NULL && rgb_mutex != NULL);
}

static void rgb_led_task(void *pvParameters)
{
    uint32_t step = 0;
    rgb_color_t blink_colors[] = {
        RGB_RED, RGB_GREEN, RGB_BLUE, RGB_YELLOW, RGB_CYAN, RGB_MAGENTA, RGB_WHITE
    };
    uint8_t color_index = 0;
    bool led_state = false;
    
    ESP_LOGI(TAG, "RGB LED animation task started");
    
    while (1) {
        switch (current_mode) {
            case RGB_MODE_BLINK:
                if (led_state) {
                    rgb_led_set_color(blink_colors[color_index]);
                    color_index = (color_index + 1) % (sizeof(blink_colors) / sizeof(blink_colors[0]));
                } else {
                    rgb_led_off();
                }
                led_state = !led_state;
                break;
                
            case RGB_MODE_RAINBOW:
                rainbow_effect(step);
                break;
                
            case RGB_MODE_BREATHE:
                breathe_effect(step, (rgb_color_t)RGB_BLUE);
                break;
                
            case RGB_MODE_FLIGHT_STATUS:
                // Flight controller status indication
                if (step % 4 == 0) {
                    rgb_led_set_color((rgb_color_t)RGB_GREEN); // Armed/Ready
                } else if (step % 4 == 1) {
                    rgb_led_set_color((rgb_color_t)RGB_BLUE);  // GPS lock
                } else if (step % 4 == 2) {
                    rgb_led_set_color((rgb_color_t)RGB_YELLOW); // Warning
                } else {
                    rgb_led_off();
                }
                break;
                
            default:
                break;
        }
        
        step++;
        vTaskDelay(pdMS_TO_TICKS(animation_period));
    }
}

static void rainbow_effect(uint32_t step)
{
    // Generate rainbow colors using HSV to RGB conversion
    float hue = (step * 360.0f / 50.0f); // Full rainbow cycle in 50 steps
    if (hue >= 360.0f) hue -= 360.0f;
    
    float s = 1.0f; // Saturation
    float v = 1.0f; // Value (brightness)
    
    float c = v * s;
    float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float r, g, b;
    
    if (hue < 60.0f) {
        r = c; g = x; b = 0;
    } else if (hue < 120.0f) {
        r = x; g = c; b = 0;
    } else if (hue < 180.0f) {
        r = 0; g = c; b = x;
    } else if (hue < 240.0f) {
        r = 0; g = x; b = c;
    } else if (hue < 300.0f) {
        r = x; g = 0; b = c;
    } else {
        r = c; g = 0; b = x;
    }
    
    rgb_color_t color = {
        .red = (uint8_t)((r + m) * 255),
        .green = (uint8_t)((g + m) * 255),
        .blue = (uint8_t)((b + m) * 255)
    };
    
    rgb_led_set_color(color);
}

static void breathe_effect(uint32_t step, rgb_color_t base_color)
{
    // Breathing effect using sine wave
    float brightness = (sinf(step * 2.0f * M_PI / 50.0f) + 1.0f) / 2.0f; // 0 to 1
    
    rgb_color_t color = {
        .red = (uint8_t)(base_color.red * brightness),
        .green = (uint8_t)(base_color.green * brightness),
        .blue = (uint8_t)(base_color.blue * brightness)
    };
    
    rgb_led_set_color(color);
} 