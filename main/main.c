// Borda Academy 2024 
// Embedded Systems Developer Intern & New Graduate Candidate Assignment
// Author: Denis Davidoglu
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "portmacro.h"
#include "driver/gptimer.h"

#include "config.h"
#include "i2c.h"
#include "filter.h"
#include "queue.h"

// FreeRTOS handles
QueueHandle_t humidity_queue;
QueueHandle_t temperature_queue;
QueueHandle_t heart_queue;
TaskHandle_t collect_queues_task_handle;

void app_main(void) {
    if (i2c_init()) return;
    humidity_queue = xQueueCreate(128, sizeof(float));
    temperature_queue = xQueueCreate(128, sizeof(float));
    heart_queue = xQueueCreate(128, sizeof(float));
    xTaskCreate(collect_queues_task, "COLLECT_QUEUES", 4096, NULL, 0, &collect_queues_task_handle);
    sensors_configure();
    sensors_read();
}
