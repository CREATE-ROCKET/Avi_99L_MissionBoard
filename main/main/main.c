#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
    while (true)
    {
        printf("Hello World from ESP32 S3\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}
