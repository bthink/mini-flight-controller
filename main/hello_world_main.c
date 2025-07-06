#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "lvgl.h"
#include "config.h"
#include "rgb_led.h"
#include "wifi_manager.h"
#include "aircraft_tracker.h"

static const char *TAG = "lcd_example";

static lv_disp_draw_buf_t disp_buf;    // Contains internal graphic buffer(s)
static lv_disp_drv_t disp_drv;         // Contains callback functions
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;
static lv_obj_t *status_label = NULL; // Label for WiFi status

// Demo counter for RGB LED effects
// static uint32_t demo_counter = 0; // Currently unused

// Function declarations
static void rgb_led_demo_task(void *pvParameters);
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

static void lvgl_tick_task(void *arg)
{
    while (1) {
        lv_tick_inc(portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void rgb_led_demo_task(void *pvParameters)
{
    // ESP_LOGI(TAG, "RGB LED demo task started");
    
    // Check if RGB LED is available
    if (!rgb_led_is_initialized()) {
        // ESP_LOGW(TAG, "RGB LED not initialized, demo task exiting");
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
        // ESP_LOGI(TAG, "RGB LED Demo: %s mode for 10 seconds", mode_names[mode_index]);
        
        // Set current mode (function handles null checks internally)
        esp_err_t ret = rgb_led_set_mode(demo_modes[mode_index], demo_periods[mode_index]);
        if (ret != ESP_OK) {
            // ESP_LOGW(TAG, "RGB LED not available, demo task exiting");
            vTaskDelete(NULL); // Delete this task
            return;
        }
        
        // Wait 10 seconds
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Move to next mode
        mode_index = (mode_index + 1) % num_modes;
    }
}

static void update_display_status(void)
{
    if (status_label == NULL) return;
    
    char status_text[512];
    char ip_str[16];
    int8_t rssi;
    
    wifi_status_t wifi_status = wifi_manager_get_status();
    
    // Get aircraft data
    aircraft_data_t aircraft_list[5];  // Show max 5 aircraft on screen
    uint8_t aircraft_count = 0;
    tracker_stats_t tracker_stats;
    
    if (aircraft_tracker_is_running()) {
        aircraft_tracker_get_aircraft(aircraft_list, 5, &aircraft_count);
        aircraft_tracker_get_stats(&tracker_stats);
    }
    
    char wifi_info[128] = "";
    switch (wifi_status) {
        case WIFI_STATUS_CONNECTED:
            if (wifi_manager_get_info(ip_str, &rssi) == ESP_OK) {
                snprintf(wifi_info, sizeof(wifi_info), "WiFi: %s (%d dBm)", ip_str, rssi);
            } else {
                snprintf(wifi_info, sizeof(wifi_info), "WiFi: Connected");
            }
            break;
        case WIFI_STATUS_CONNECTING:
            snprintf(wifi_info, sizeof(wifi_info), "WiFi: Connecting...");
            break;
        case WIFI_STATUS_FAILED:
            snprintf(wifi_info, sizeof(wifi_info), "WiFi: Connection failed");
            break;
        default:
            snprintf(wifi_info, sizeof(wifi_info), "WiFi: Disconnected");
            break;
    }
    
    // Direction arrows for 8-direction compass
    const char* direction_arrows[] = {"↑", "↗", "→", "↘", "↓", "↙", "←", "↖"};
    
    if (aircraft_tracker_is_running() && aircraft_count > 0) {
        // Show aircraft tracker data
        char aircraft_info[300] = "";
        char temp_line[80];
        
        for (int i = 0; i < aircraft_count && i < 3; i++) {  // Show max 3 aircraft
            const aircraft_data_t *ac = &aircraft_list[i];
            
            if (strlen(ac->callsign) > 0) {
                snprintf(temp_line, sizeof(temp_line), "%s %s %.1fkm %s\n", 
                         direction_arrows[ac->direction], 
                         ac->callsign, 
                         ac->distance_km,
                         ac->on_ground ? "GND" : "AIR");
            } else {
                snprintf(temp_line, sizeof(temp_line), "%s %s %.1fkm %s\n", 
                         direction_arrows[ac->direction], 
                         ac->icao24, 
                         ac->distance_km,
                         ac->on_ground ? "GND" : "AIR");
            }
            
            strncat(aircraft_info, temp_line, sizeof(aircraft_info) - strlen(aircraft_info) - 1);
        }
        
        snprintf(status_text, sizeof(status_text), 
                 "AIRCRAFT TRACKER\n"
                 "%s\n"
                 "Found %d aircraft:\n"
                 "%s"
                 "Requests: %d/%d", 
                 wifi_info,
                 aircraft_count,
                 aircraft_info,
                 (int)tracker_stats.successful_requests,
                 (int)tracker_stats.total_requests);
    } else if (aircraft_tracker_is_running()) {
        // Tracker running but no aircraft
        snprintf(status_text, sizeof(status_text), 
                 "AIRCRAFT TRACKER\n"
                 "%s\n"
                 "Searching for aircraft...\n"
                 "Radius: %d km\n"
                 "Requests: %d/%d", 
                 wifi_info,
                 TRACKER_RADIUS_KM,
                 (int)tracker_stats.successful_requests,
                 (int)tracker_stats.total_requests);
    } else {
        // Tracker not running
        snprintf(status_text, sizeof(status_text), 
                 "FLIGHT CONTROLLER\n"
                 "%s\n"
                 "Aircraft tracker: OFF\n"
                 "RGB LED Active", 
                 wifi_info);
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

    // Allocate two buffers for LVGL drawing
    buf1 = heap_caps_malloc(LCD_V_RES * LCD_H_RES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    buf2 = heap_caps_malloc(LCD_V_RES * LCD_H_RES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);

    // Initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_V_RES * LCD_H_RES);

    // Register display driver to LVGL
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_V_RES;
    disp_drv.ver_res = LCD_H_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp_drv.full_refresh = 1;
    lv_disp_drv_register(&disp_drv);

    // Create a task to handle LVGL ticks
    xTaskCreate(lvgl_tick_task, "lvgl_tick", 4096, NULL, 1, NULL);

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
    
    // Initialize RGB LED
    esp_err_t rgb_ret = rgb_led_init();
    if (rgb_ret == ESP_OK) {
        // ESP_LOGI(TAG, "RGB LED initialized successfully");
        
        // Start RGB LED demo task
        xTaskCreate(rgb_led_demo_task, "rgb_led_demo_task", 2048, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "RGB LED initialization failed: %s", esp_err_to_name(rgb_ret));
        ESP_LOGW(TAG, "Continuing without RGB LED functionality");
    }
    
    // Initialize Aircraft Tracker
    ESP_LOGI(TAG, "Initializing Aircraft Tracker...");
    esp_err_t tracker_ret = aircraft_tracker_init();
    if (tracker_ret == ESP_OK) {
        ESP_LOGI(TAG, "Aircraft tracker initialized successfully");
        
        // Start aircraft tracking if WiFi is connected
        wifi_status_t current_wifi_status = wifi_manager_get_status();
        ESP_LOGI(TAG, "Current WiFi status: %d", current_wifi_status);
        
        if (current_wifi_status == WIFI_STATUS_CONNECTED) {
            ESP_LOGI(TAG, "WiFi is connected, starting aircraft tracker immediately");
            esp_err_t start_ret = aircraft_tracker_start();
            if (start_ret == ESP_OK) {
                ESP_LOGI(TAG, "Aircraft tracker started successfully");
            } else {
                ESP_LOGW(TAG, "Failed to start aircraft tracker: %s", esp_err_to_name(start_ret));
            }
        } else {
            ESP_LOGI(TAG, "WiFi not connected yet (status=%d), aircraft tracker will start automatically when WiFi connects", current_wifi_status);
        }
    } else {
        ESP_LOGE(TAG, "Aircraft tracker initialization failed: %s", esp_err_to_name(tracker_ret));
        ESP_LOGW(TAG, "Continuing without aircraft tracking functionality");
    }

    uint32_t display_update_counter = 0;
    bool tracker_auto_started = false;
    
    while (1) {
        lv_timer_handler();
        
        // Auto-start aircraft tracker when WiFi connects
        wifi_status_t current_status = wifi_manager_get_status();
        bool tracker_running = aircraft_tracker_is_running();
        
        if (!tracker_auto_started && 
            current_status == WIFI_STATUS_CONNECTED && 
            !tracker_running) {
            
            ESP_LOGI(TAG, "WiFi connected (status=%d), tracker_running=%d, starting aircraft tracker", current_status, tracker_running);
            esp_err_t start_ret = aircraft_tracker_start();
            if (start_ret == ESP_OK) {
                ESP_LOGI(TAG, "Aircraft tracker auto-started successfully");
                tracker_auto_started = true;
            } else {
                ESP_LOGW(TAG, "Failed to auto-start aircraft tracker: %s", esp_err_to_name(start_ret));
            }
        }
        
        // Debug info every 10 seconds (2000 * 5ms = 10s)
        static uint32_t debug_counter = 0;
        debug_counter++;
        if (debug_counter >= 2000) {
            ESP_LOGI(TAG, "DEBUG: WiFi status=%d, tracker_running=%d, auto_started=%d", 
                     current_status, tracker_running, tracker_auto_started);
            debug_counter = 0;
        }
        
        // Reset auto-start flag if WiFi disconnects
        if (tracker_auto_started && wifi_manager_get_status() != WIFI_STATUS_CONNECTED) {
            tracker_auto_started = false;
        }
        
        // Update display status every 2 seconds (400 * 5ms = 2000ms)
        display_update_counter++;
        if (display_update_counter >= 400) {
            update_display_status();
            display_update_counter = 0;
        }
        
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
