idf_component_register(SRCS "hello_world_main.c"
                           "rgb_led.c"
                           "wifi_manager.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_lcd esp_driver_gpio esp_driver_spi lvgl led_strip esp_wifi nvs_flash esp_netif esp_event)
