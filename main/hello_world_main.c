#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "lvgl.h"
#include "aircraft_tracker.h"
#include "config.h"
#include "rgb_led.h"
#include "wifi_manager.h"

static const char *TAG = "lcd_example";

static lv_disp_draw_buf_t disp_buf;    // Contains internal graphic buffer(s)
static lv_disp_drv_t disp_drv;         // Contains callback functions
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;
static lv_obj_t *status_label = NULL; // Label for WiFi status
static SemaphoreHandle_t s_aircraft_mutex = NULL;
static aircraft_info_t s_latest_aircraft = {0};
static bool s_aircraft_data_ready = false;

#define LVGL_DRAW_BUF_LINES 40

// Demo counter for RGB LED effects
// static uint32_t demo_counter = 0; // Currently unused

// Function declarations
static void rgb_led_demo_task(void *pvParameters);
static void aircraft_tracker_task(void *pvParameters);
static void update_display_status(void);

// ST7789 initialization commands
typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t data_bytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

static const lcd_init_cmd_t st_init_cmds[] = {
    {0x36, {0x70}, 1},
    {0x3A, {0x05}, 1},
    {0xB2, {0x0C, 0x0C, 0x00, 0x33, 0x33}, 5},
    {0xB7, {0x35}, 1},
    {0xBB, {0x19}, 1},
    {0xC0, {0x2C}, 1},
    {0xC2, {0x01}, 1},
    {0xC3, {0x12}, 1},
    {0xC4, {0x20}, 1},
    {0xC6, {0x0F}, 1},
    {0xD0, {0xA4, 0xA1}, 2},
    {0xE0, {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23}, 14},
    {0xE1, {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}, 14},
    {0x21, {0}, 0},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff}
};

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

static void st7789_send_init_commands(esp_lcd_panel_io_handle_t io_handle)
{
    int cmd = 0;
    const lcd_init_cmd_t* lcd_init_cmds = st_init_cmds;

    while (lcd_init_cmds[cmd].data_bytes != 0xff) {
        esp_lcd_panel_io_tx_param(io_handle, lcd_init_cmds[cmd].cmd, lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].data_bytes & 0x7f);
        if (lcd_init_cmds[cmd].data_bytes & 0x80) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        cmd++;
    }
}

static void rgb_led_demo_task(void *pvParameters)
{
    ESP_LOGI(TAG, "RGB LED demo task started");
    
    // Check if RGB LED is available
    if (!rgb_led_is_initialized()) {
        ESP_LOGW(TAG, "RGB LED not initialized, demo task exiting");
        vTaskDelete(NULL);
        return;
    }
    
    const rgb_led_mode_t demo_modes[] = {
        RGB_MODE_RAINBOW,
        RGB_MODE_BLINK, 
        RGB_MODE_BREATHE,
        RGB_MODE_FLIGHT_STATUS
    };
    
    const uint32_t demo_periods[] = {
        100,  // Rainbow - fast
        500,  // Blink - medium
        150,  // Breathe - slow breathing
        800   // Flight status - slower
    };
    
    const char* mode_names[] = {
        "Rainbow",
        "Blink",
        "Breathe", 
        "Flight Status"
    };
    
    uint8_t mode_index = 0;
    const uint8_t num_modes = sizeof(demo_modes) / sizeof(demo_modes[0]);
    
    while (1) {
        ESP_LOGI(TAG, "RGB LED Demo: %s mode for 10 seconds", mode_names[mode_index]);
        
        // Set current mode (function handles null checks internally)
        esp_err_t ret = rgb_led_set_mode(demo_modes[mode_index], demo_periods[mode_index]);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "RGB LED not available, demo task exiting");
            vTaskDelete(NULL); // Delete this task
            return;
        }
        
        // Wait 10 seconds
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Move to next mode
        mode_index = (mode_index + 1) % num_modes;
    }
}

static void aircraft_tracker_task(void *pvParameters)
{
    while (1) {
        if (wifi_manager_is_connected()) {
            aircraft_info_t info = {0};
            esp_err_t ret = aircraft_tracker_fetch_nearest(&info);

            if (s_aircraft_mutex != NULL && xSemaphoreTake(s_aircraft_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                if (ret == ESP_OK && info.valid) {
                    s_latest_aircraft = info;
                    s_aircraft_data_ready = true;
                    ESP_LOGI(TAG, "Nearest aircraft: %s %.1f km alt %d m heading %d",
                             info.callsign, info.distance_km, info.altitude_m, info.heading_deg);
                } else {
                    s_aircraft_data_ready = false;
                    ESP_LOGI(TAG, "No nearby aircraft or API unavailable");
                }
                xSemaphoreGive(s_aircraft_mutex);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(TRACKER_REFRESH_SEC * 1000));
    }
}

static void update_display_status(void)
{
    if (status_label == NULL) return;
    
    char status_text[256];
    char ip_str[16];
    int8_t rssi;
    aircraft_info_t aircraft = {0};
    bool aircraft_ready = false;
    
    wifi_status_t wifi_status = wifi_manager_get_status();

    if (s_aircraft_mutex != NULL && xSemaphoreTake(s_aircraft_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        aircraft = s_latest_aircraft;
        aircraft_ready = s_aircraft_data_ready;
        xSemaphoreGive(s_aircraft_mutex);
    }
    
    switch (wifi_status) {
        case WIFI_STATUS_CONNECTED:
            if (wifi_manager_get_info(ip_str, &rssi) == ESP_OK) {
                snprintf(status_text, sizeof(status_text), 
                         "FLIGHT CONTROLLER\n"
                         "WiFi: Connected\n"
                         "IP: %s\n"
                         "RSSI: %d dBm\n", 
                         ip_str, rssi);
            } else {
                snprintf(status_text, sizeof(status_text), 
                         "FLIGHT CONTROLLER\n"
                         "WiFi: Connected\n");
            }
            if (aircraft_ready && aircraft.valid) {
                snprintf(status_text + strlen(status_text), sizeof(status_text) - strlen(status_text),
                         "Aircraft: %s\n"
                         "Dist: %.1f km Alt: %d m\n"
                         "Head: %d deg",
                         aircraft.callsign,
                         aircraft.distance_km,
                         aircraft.altitude_m,
                         aircraft.heading_deg);
            } else {
                snprintf(status_text + strlen(status_text), sizeof(status_text) - strlen(status_text),
                         "Aircraft: no data");
            }
            break;
        case WIFI_STATUS_CONNECTING:
            snprintf(status_text, sizeof(status_text), 
                     "FLIGHT CONTROLLER\n"
                     "WiFi: Connecting...\n"
                     "Aircraft: waiting");
            break;
        case WIFI_STATUS_FAILED:
            snprintf(status_text, sizeof(status_text), 
                     "FLIGHT CONTROLLER\n"
                     "WiFi: Failed to connect\n"
                     "Aircraft: unavailable");
            break;
        default:
            snprintf(status_text, sizeof(status_text), 
                     "FLIGHT CONTROLLER\n"
                     "WiFi: Disconnected\n"
                     "Aircraft: unavailable");
            break;
    }
    
    lv_label_set_text(status_label, status_text);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t)
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7789 panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Send custom initialization commands
    st7789_send_init_commands(io_handle);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Set display orientation and color mode
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    
    // Additional display configuration
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 35));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Initialize LCD backlight
    gpio_config_t bk_gpio_config = {
        .pin_bit_mask = 1ULL << PIN_NUM_LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
    
    // Turn on display and backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_LCD_BL, LCD_BK_LIGHT_ON_LEVEL);

    // Initialize LVGL
    lv_init();

    // Allocate two partial buffers for LVGL drawing
    const size_t draw_buf_pixels = LCD_V_RES * LVGL_DRAW_BUF_LINES;
    buf1 = heap_caps_malloc(draw_buf_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    buf2 = heap_caps_malloc(draw_buf_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);

    // Initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, draw_buf_pixels);

    // Register display driver to LVGL
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_V_RES;
    disp_drv.ver_res = LCD_H_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    // Create a label for status display
    status_label = lv_label_create(lv_scr_act());
    lv_label_set_text(status_label, "FLIGHT CONTROLLER\nInitializing...");
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(status_label, lv_color_white(), 0);
    lv_obj_set_style_text_align(status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(status_label);

    ESP_LOGI(TAG, "Display setup completed");
    
    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    esp_err_t wifi_ret = wifi_manager_init();
    if (wifi_ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi initialized successfully");
    } else {
        ESP_LOGE(TAG, "WiFi initialization failed: %s", esp_err_to_name(wifi_ret));
    }
    
    // Update display with initial status
    update_display_status();

    s_aircraft_mutex = xSemaphoreCreateMutex();
    if (s_aircraft_mutex != NULL) {
        xTaskCreate(aircraft_tracker_task, "aircraft_tracker", 16384, NULL, 2, NULL);
    } else {
        ESP_LOGE(TAG, "Failed to create aircraft tracker mutex");
    }
    
    // Initialize RGB LED
    esp_err_t rgb_ret = rgb_led_init();
    if (rgb_ret == ESP_OK) {
        ESP_LOGI(TAG, "RGB LED initialized successfully");
        
        // Start with rainbow effect
        rgb_led_set_mode(RGB_MODE_RAINBOW, 100);
        
        // Create a task for LED demo
        xTaskCreate(rgb_led_demo_task, "rgb_demo", 4096, NULL, 3, NULL);
    } else {
        ESP_LOGE(TAG, "RGB LED initialization failed: %s", esp_err_to_name(rgb_ret));
        ESP_LOGW(TAG, "Continuing without RGB LED functionality");
    }

    uint32_t display_update_counter = 0;
    while (1) {
        uint32_t wait_ms = lv_timer_handler();
        if (wait_ms < 5) {
            wait_ms = 5;
        } else if (wait_ms > 20) {
            wait_ms = 20;
        }
        
        // Update display status every ~2 seconds
        display_update_counter += wait_ms;
        if (display_update_counter >= 2000) {
            update_display_status();
            display_update_counter = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        lv_tick_inc(wait_ms);
    }
}
