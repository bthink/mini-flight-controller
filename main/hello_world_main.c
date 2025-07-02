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

static const char *TAG = "lcd_example";

static lv_disp_draw_buf_t disp_buf;    // Contains internal graphic buffer(s)
static lv_disp_drv_t disp_drv;         // Contains callback functions
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;

// Demo counter for RGB LED effects
static uint32_t demo_counter = 0;

// Function declarations
static void rgb_led_demo_task(void *pvParameters);

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
    ESP_LOGI(TAG, "RGB LED demo task started");
    
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
        
        // Set current mode
        rgb_led_set_mode(demo_modes[mode_index], demo_periods[mode_index]);
        
        // Wait 10 seconds
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // Move to next mode
        mode_index = (mode_index + 1) % num_modes;
    }
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

    // Create a label and set its text
    lv_obj_t *label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "FLIGHT CONTROLLER\nRGB LED Demo Active\nWatch the LED!");
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(label);

    ESP_LOGI(TAG, "Display setup completed");
    
    // Initialize RGB LED
    ESP_ERROR_CHECK(rgb_led_init());
    ESP_LOGI(TAG, "RGB LED initialized");
    
    // Start with rainbow effect
    ESP_ERROR_CHECK(rgb_led_set_mode(RGB_MODE_RAINBOW, 100));
    
    // Create a task for LED demo
    xTaskCreate(rgb_led_demo_task, "rgb_demo", 4096, NULL, 3, NULL);

    while (1) {
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
