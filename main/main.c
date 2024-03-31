// Borda Academy 2024 
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
#include "esp_random.h"
#include "portmacro.h"
#include "xtensa/hal.h"
#include "driver/gptimer.h"
#include <math.h>
#include "i2c.h"
#include "config.h"

// My type definitions
typedef struct {
    char name[16];
    float stddev;
    float max;
    float min;
    float median;
} ble_packet_t;

// My functions
void swap_f(float*, float*); 
float find_rank(float*, uint16_t, uint16_t);
void filter_sensor_value(float*, uint16_t, uint16_t); 
float find_stddev(float*, uint16_t);
static bool output_irq(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
void collect_queues_task(void*); 
void ble_transmit_packet(ble_packet_t*, uint8_t);
void app_main(void);

// My define's

#define find_median(array, size) find_rank((array), (size), (size)/2)
#define find_min(array, size) find_rank((array), (size), 0)
#define find_max(array, size) find_rank((array), (size), (size)-1)

// Timer related definitions
gptimer_handle_t output_timer_handle = NULL;
gptimer_config_t output_timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000000, 
};
gptimer_alarm_config_t output_alarm_config = {
    .reload_count = 0, 
    .alarm_count = 1000000*BLE_OUTPUT_RATE, // period = 1s 
    .flags.auto_reload_on_alarm = true, 
};

gptimer_event_callbacks_t output_callbacks = {
    .on_alarm = output_irq,
};

// FreeRTOS handles
QueueHandle_t humidity_queue;
QueueHandle_t temperature_queue;
QueueHandle_t heart_queue;
TaskHandle_t collect_queues_task_handle;
SemaphoreHandle_t output_sema_handle;

void swap_f(float *a, float *b) {
    float c = *a;
    *a = *b;
    *b = c;
}

float find_rank(float *array, uint16_t size, uint16_t k) {
    if (size == 0) return 0;
    if (size == 1) return array[0];
    // Place randomly selected pivot to the beginning of array
    swap_f(array, array + (esp_random()%size));
    uint16_t pointer1 = 1;
    uint16_t pointer2 = size-1;

    // Partition
    while (pointer1 <= pointer2) {
        if (array[pointer1] < array[0]) {
            pointer1++;
        } else {
            swap_f(array + pointer1, array + pointer2);
            pointer2--;
        }
    }
    // Put the pivot in the right place
    swap_f(array, array + pointer2);
    // Recurse
    if (pointer2 >= k)
        return find_rank(array, pointer2 + 1, k);
    return find_rank(array + pointer2 + 1, size - pointer2 - 1, k - pointer2 - 1);
}

void filter_sensor_value(float *array, uint16_t size, uint16_t window_size) {
    float filtered_array[size];
    for (int i = 0; i < size; i++) {
        float window[2*window_size + 1];
        window[window_size] = array[i];
        for (int j = 0; j < window_size; j++) {
            window[j] = array[(i - 1 - j + size) % size];
            window[window_size + j + 1] = array[(i + j) % size];
        }
        filtered_array[i] = find_median(window, 2*window_size + 1);
    }
    xthal_memcpy(array, filtered_array, sizeof(float)*size);
}

float find_stddev(float *array, uint16_t size) {
    float result = 0;
    if (size != 0) {
        float mean = 0;
        for (uint16_t i = 0; i < size; i++)
            mean += array[i];
        mean /= (float)size;
        for (uint16_t i = 0; i < size; i++)
            result += powf((array[i] - mean), 2); 
        result /= (float)size;
        result = sqrt(result);
    }
    return result;
}

static bool output_irq(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    xSemaphoreGiveFromISR(output_sema_handle, NULL);
    return true;
}

void collect_queues_task(void *args) {
    const uint16_t t_max_size = 10;
    const uint16_t hu_max_size = 10;
    const uint16_t he_max_size = 10;
    float temperature_buffer[t_max_size];
    float humidity_buffer[hu_max_size];
    float heart_buffer[he_max_size];
    uint16_t t_size = 0;
    uint16_t hu_size = 0;
    uint16_t he_size = 0;
    uint16_t i = 0, j = 0, k = 0;

    output_sema_handle = xSemaphoreCreateBinary();
    gptimer_new_timer(&output_timer_config, &output_timer_handle);
    gptimer_set_alarm_action(output_timer_handle, &output_alarm_config);
    gptimer_register_event_callbacks(output_timer_handle, &output_callbacks, NULL);
    gptimer_enable(output_timer_handle);
    gptimer_start(output_timer_handle);

    while (1) {
        if (xQueueReceive(temperature_queue, temperature_buffer + i, 0) == pdTRUE) {
            i = (i + 1) % t_max_size;
            if (t_size < t_max_size) t_size++;
        }
        if (xQueueReceive(humidity_queue, humidity_buffer + j, 0) == pdTRUE) {
            j = (j + 1) % hu_max_size;
            if (hu_size < hu_max_size) hu_size++;
        }
        if (xQueueReceive(heart_queue, heart_buffer + k, 0) == pdTRUE) {
            k = (k + 1) % he_max_size;
            if (he_size < he_max_size) he_size++;
        }
        if (xSemaphoreTake(output_sema_handle, 0) == pdTRUE) {
            filter_sensor_value(temperature_buffer, t_size, 1);
            filter_sensor_value(humidity_buffer, hu_size, 1);
            filter_sensor_value(heart_buffer, he_size, 1);
            
            ble_packet_t packet[3] = {
                {
                    .name = "Temperature",
                    .stddev = find_stddev(temperature_buffer, t_size),
                    .max = find_max(temperature_buffer, t_size),
                    .min = find_min(temperature_buffer, t_size),
                    .median = find_median(temperature_buffer, t_size),
                },
                {
                    .name = "Humidity",
                    .stddev = find_stddev(humidity_buffer, hu_size),
                    .max = find_max(humidity_buffer, hu_size),
                    .min = find_min(humidity_buffer, hu_size),
                    .median = find_median(humidity_buffer, hu_size),
                },
                {
                    .name = "Heart rate",
                    .stddev = find_stddev(heart_buffer, he_size),
                    .max = find_max(heart_buffer, he_size),
                    .min = find_min(heart_buffer, he_size),
                    .median = find_median(heart_buffer, he_size),
                },
            };
            ble_transmit_packet(packet, 3);
        }
    }
}

void ble_transmit_packet(ble_packet_t *packet, uint8_t size) {
    printf("| %16s | %16s | %16s | %16s | %16s |\n", "Name", "Stddev", "Max", "Min", "Median");
    printf("+------------------+------------------+------------------+------------------+------------------+\n");
    for (uint8_t i = 0; i < size; i++) {
        printf("| %16s | %16f | %16f | %16f | %16f |\n", packet[i].name, packet[i].stddev, packet[i].max, packet[i].min, packet[i].median);
    }
    printf("\n");
}

void app_main(void) {
    if (i2c_init()) return;
    humidity_queue = xQueueCreate(128, sizeof(float));
    temperature_queue = xQueueCreate(128, sizeof(float));
    heart_queue = xQueueCreate(128, sizeof(float));
    xTaskCreate(collect_queues_task, "COLLECT_QUEUES", 4096, NULL, 0, &collect_queues_task_handle);
    sensors_configure();
    sensors_read();
}
