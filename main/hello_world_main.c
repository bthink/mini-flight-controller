#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    while (1) {
        printf("👋 Hello from ESP32-C6! %lu\n", (unsigned long)xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
