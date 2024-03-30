#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_intr_alloc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "hal/i2c_types.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "esp_random.h"
#include "portmacro.h"
#include "xtensa/hal.h"
#include "driver/gptimer.h"

#define DELAYMS(x) vTaskDelay((x) / portTICK_PERIOD_MS)
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 23
#define MAX30100_INT_PIN 19

i2c_master_bus_config_t i2c_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = -1,
    .scl_io_num = I2C_SCL_PIN,
    .sda_io_num = I2C_SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};

i2c_master_bus_handle_t bus_handle;

typedef enum {
    SHT3X,
    BMP180,
    MAX30100,
} sensor_id_t;

typedef enum {
    AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD,
} bmp180_eeprom_t;

i2c_device_config_t dev_config[] = {
    { 
    .dev_addr_length = I2C_ADDR_BIT_7,
    .device_address = 0x44,
    .scl_speed_hz = 100000,
    },
    {
    .dev_addr_length = I2C_ADDR_BIT_7,
    .device_address = 0x77,
    .scl_speed_hz = 100000,
    },
    {
    .dev_addr_length = I2C_ADDR_BIT_7, 
    .device_address = 0x57,
    .scl_speed_hz = 100000,
    },
};
static int dev_count = sizeof(dev_config)/sizeof(i2c_device_config_t);

i2c_master_dev_handle_t dev_handle[10];

gptimer_handle_t output_timer_handle = NULL;
gptimer_config_t output_timer_config = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 1000*1000, // 1Hz
};
gptimer_alarm_config_t output_alarm_config = {
    .reload_count = 0, 
    .alarm_count = 1000*1000*1, // period = 1s 
    .flags.auto_reload_on_alarm = true, 
};

static bool output(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx); 

gptimer_event_callbacks_t output_callbacks = {
    .on_alarm = output,
};

QueueHandle_t humidity_queue;
QueueHandle_t temperature_queue;
QueueHandle_t heart_queue;
TaskHandle_t collect_queues_task_handle;
SemaphoreHandle_t max30100_sema_handle;
SemaphoreHandle_t output_sema_handle;

#define SHT3X_INITIAL_CRC 0xff

uint16_t bmp180_eeprom[11] = {};

void max30100_irq(void *arg) {
    xSemaphoreGiveFromISR(max30100_sema_handle, NULL);
}

void sensors_configure() {
    uint8_t command[2] = {};
    uint8_t temp[6] = {};
    int i = 0;
    
    // BMP180
    // Saving EEPROM contents
    i = 0;
    for (command[0] = 0xAA; command[0] < 0xBF; command[0] += 2) {
        i2c_master_transmit(dev_handle[BMP180], command, 1, -1); 
        i2c_master_receive(dev_handle[BMP180], temp, 2, -1);
        bmp180_eeprom[i] = (((uint16_t)temp[0]) << 8) | temp[1];
        printf("%d\n", bmp180_eeprom[i]);
        i++;  
    }

    // MAX30100
    command[0] = 0x06; // MODE address
    command[1] = 0x02; // Heart rate only enabled
    i2c_master_transmit(dev_handle[MAX30100], command, 2, -1);

    command[0] = 0x07; // SPO2 configuration
    command[1] = 0x0F; // highest resolution (16 bits)
    i2c_master_transmit(dev_handle[MAX30100], command, 2, -1);

    command[0] = 0x01; // Enabling interrupt
    command[1] = (1 << 5); // ENB_HR_RDY
    i2c_master_transmit(dev_handle[MAX30100], command, 2, -1);

    command[0] = 0x00; // Clearing pending interrupt
    i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
    i2c_master_receive(dev_handle[MAX30100], command, 1, -1);
    printf("Interrupt register: 0x%x\n", command[0]);

    gpio_set_direction(MAX30100_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_intr_type(MAX30100_INT_PIN, GPIO_INTR_NEGEDGE); 
    gpio_install_isr_service(ESP_INTR_FLAG_SHARED);
    gpio_isr_handler_add(MAX30100_INT_PIN, max30100_irq, NULL);
    gpio_intr_enable(MAX30100_INT_PIN);
}

uint8_t sht3x_crc(uint16_t data, uint8_t crc) {
    return SHT3X_INITIAL_CRC;
    // Code below does not comply with CRC-8, to be changed
    uint32_t polynomial = ((1 << 8) | (1 << 5) | (1 << 4) | (1 << 0)) << 15;
    uint32_t padded_data = ((uint32_t)data << 8) | (uint32_t)crc;
    for (int i = 23; i >= 8; i--) {
        if ((1 << i) & padded_data) 
            padded_data = (padded_data ^ polynomial);
        polynomial >>= 1;
    }
    return padded_data;  
}

void sensors_read(void) {
    uint8_t command[4] = {};
    uint8_t temp[10] = {};
    uint8_t max30100_fifo_wr_ptr = 0; 
    uint8_t max30100_fifo_rd_ptr = 0;

    while (1) {
        // SHT3X
        // Reading humidity
        command[0] = 0x24; 
        command[1] = 0x00; 
        i2c_master_transmit(dev_handle[SHT3X], command, 2, -1); 
        DELAYMS(2);
        i2c_master_receive(dev_handle[SHT3X], temp, 6, -1);
        uint16_t sht3x_srh = (((uint16_t)temp[3]) << 8) | temp[4];
        float sht3x_rh = 0;

        // Checksum 
        if (sht3x_crc(sht3x_srh, temp[5] == SHT3X_INITIAL_CRC)) {
            // Relative humiditiy
            sht3x_rh = 100.0 * (float)(sht3x_srh) / (float)((1 << 16) - 1);
        }

        // BMP180 
        command[0] = 0xF4;
        command[1] = 0x2E;
        i2c_master_transmit(dev_handle[BMP180], &command[0], 2, -1); 
        DELAYMS(5);
        command[0] = 0xF6;
        i2c_master_transmit(dev_handle[BMP180], &command[0], 1, -1); 
        i2c_master_receive(dev_handle[BMP180], temp, 3, -1);
        // Uncompensated temperature value
        long bmp180_ut =  (((uint16_t)temp[0]) << 8) | (uint16_t)temp[1];
        long x1 = ((bmp180_ut - (unsigned short)bmp180_eeprom[AC6])*(unsigned short)bmp180_eeprom[AC5]) / (1 << 15);
        long x2 = ((short)bmp180_eeprom[MC] << 11) / (x1 + (short)bmp180_eeprom[MD]);
        long b5 = x1 + x2;
        // True temperature
        float bmp180_t = (float)((b5 + 8) / (1 << 4)) / 10.0;
    
        if (sht3x_rh != 0)
            xQueueSend(humidity_queue, &sht3x_rh, portMAX_DELAY);
        xQueueSend(temperature_queue, &bmp180_t, portMAX_DELAY);
        printf("Humidity: %f%%\nTemperature: %f\n", sht3x_rh, bmp180_t);

        // MAX30100
        if (xSemaphoreTake(max30100_sema_handle, 0) == pdTRUE) {
            uint8_t num_available_samples;
            // Third transaction: Write to FIFO_RD_PTR register.
            command[0] = 0x04; // FIFO_RD_PTR address
            i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
            i2c_master_receive(dev_handle[MAX30100], &max30100_fifo_rd_ptr, 1, -1);

            // First transaction: Get the FIFO_WR_PTR:
            command[0] = 0x02; // FIFO_WR_PTR address
            i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
            i2c_master_receive(dev_handle[MAX30100], &max30100_fifo_wr_ptr, 1, -1);

            // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
            if (max30100_fifo_wr_ptr <= max30100_fifo_rd_ptr)
                max30100_fifo_wr_ptr += 0xf;
            num_available_samples = max30100_fifo_wr_ptr - max30100_fifo_rd_ptr;
           
            printf("Heart rate: ");
            for (int i = 0; i < num_available_samples; i++) {
                uint8_t temp[4];
                command[0] = 0x05; // FIFO_DATA address
                i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
                for (int j = 0; j < 4; j++) {
                    i2c_master_receive(dev_handle[MAX30100], temp+j, 1, -1);
                }
                uint16_t max30100_ir = ((uint16_t)temp[0] << 8) | temp[1];
                // In heart-rate only mode, the 3rd and 4th bytes of each sample return zeros,
                // max30100_red = ((uint16_t)temp[2] << 8) | temp[3];
                xQueueSend(heart_queue, &max30100_ir, portMAX_DELAY); 
                printf("%d ", max30100_ir);
            }
            printf("\n");

        }
        DELAYMS(500);
    }
}

void swap_f(float *a, float *b) {
    float c = *a;
    *a = *b;
    *b = c;
}

float find_median_select(float *array, uint8_t size, uint8_t k) {
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
        return find_median_select(array, pointer2 + 1, k);
    return find_median_select(array + pointer2 + 1, size - pointer2 - 1, k - pointer2 - 1);
}

float find_median(float *array, uint8_t size) {
    return find_median_select(array, size, size/2);
}

void filter_sensor_value(float *array, uint16_t size, uint8_t window_size) {
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

static bool output(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    xSemaphoreGiveFromISR(output_sema_handle, NULL);
    return true;
}

void collect_queues_task(void *args) {
    const uint16_t temperature_buffer_size = 10;
    const uint16_t humidity_buffer_size = 10;
    const uint16_t heart_buffer_size = 10;
    float temperature_buffer[temperature_buffer_size];
    float humidity_buffer[humidity_buffer_size];
    float heart_buffer[heart_buffer_size];
    uint16_t i = 0, j = 0, k = 0;

    output_sema_handle = xSemaphoreCreateBinary();
    gptimer_new_timer(&output_timer_config, &output_timer_handle);
    gptimer_set_alarm_action(output_timer_handle, &output_alarm_config);
    gptimer_register_event_callbacks(output_timer_handle, &output_callbacks, NULL);
    gptimer_enable(output_timer_handle);
    gptimer_start(output_timer_handle);

    while (1) {
        if (xQueueReceive(temperature_queue, temperature_buffer + i, 0) == pdTRUE) {
            i = (i + 1) % temperature_buffer_size;
        }
        if (xQueueReceive(humidity_queue, humidity_buffer + j, 0) == pdTRUE) {
            j = (j + 1) % humidity_buffer_size;
        }
        if (xQueueReceive(heart_queue, heart_buffer + k, 0) == pdTRUE) {
            k = (k + 1) % heart_buffer_size;
        }
        if (xSemaphoreTake(output_sema_handle, 0) == pdTRUE) {
            printf("\nHello from timer\n");
            filter_sensor_value(temperature_buffer, temperature_buffer_size, 1);
            filter_sensor_value(humidity_buffer, humidity_buffer_size, 1);
            filter_sensor_value(heart_buffer, heart_buffer_size, 1);
            printf("Temperature: ");
            for (int i = 0; i < 10; i++) {
                printf("%f ", temperature_buffer[i]);
            } 
            printf("\n");
            printf("Humidity: ");
            for (int i = 0; i < 10; i++) {
                printf("%f ", humidity_buffer[i]);
            } 
            printf("\n");
            printf("Heart rate: ");
            for (int i = 0; i < 10; i++) {
                printf("%f ", heart_buffer[i]);
            } 
            printf("\n\n");
        }
    }
}

void app_main(void) {
    if (i2c_new_master_bus(&i2c_config, &bus_handle) == ESP_OK) {
        for (int i = 0; i < dev_count; i++) {
            if (i2c_master_probe(bus_handle, dev_config[i].device_address, -1) == ESP_OK) {
                printf("Sensor no %d is available\n", i);
                i2c_master_bus_add_device(bus_handle, &dev_config[i], &dev_handle[i]);
            } else {
                printf("Application failed to start, connect all the sensors\n");
                return;
            }
        }
    }
    humidity_queue = xQueueCreate(128, sizeof(float));
    temperature_queue = xQueueCreate(128, sizeof(float));
    heart_queue = xQueueCreate(128, sizeof(float));
    xTaskCreate(collect_queues_task, "COLLECT_QUEUES", 4096, NULL, 0, &collect_queues_task_handle);
    max30100_sema_handle = xSemaphoreCreateBinary();

    sensors_configure();
    sensors_read();
}

// idf.py -B build.clang -D IDF_TOOLCHAIN=clang reconfigure
