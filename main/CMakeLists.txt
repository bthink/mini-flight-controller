idf_component_register(SRCS "hello_world_main.c"
                           "rgb_led.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_lcd esp_driver_gpio esp_driver_spi lvgl led_strip)
