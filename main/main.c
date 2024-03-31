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
#include <math.h>

// Configuration
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 23
#define MAX30100_INT_PIN 19
#define SAMPLING_RATE 1 // Hz 
#define BLE_OUTPUT_RATE 30 // seconds

// My type definitions
typedef enum {
    SHT3X, BMP180, MAX30100, 
} sensor_id_t;

typedef enum {
    AC1, AC2, AC3, AC4, AC5, AC6, B1, B2, MB, MC, MD,
} bmp180_eeprom_t;

typedef struct {
    char name[16];
    float stddev;
    float max;
    float min;
    float median;
} ble_packet_t;

// My functions
void max30100_irq(void*); 
void sensors_configure(); 
void sensors_read(); 
void swap_f(float*, float*); 
float find_rank(float*, uint16_t, uint16_t);
void filter_sensor_value(float*, uint16_t, uint16_t); 
float find_stddev(float*, uint16_t);
static bool output(gptimer_handle_t, const gptimer_alarm_event_data_t*, void*);
void collect_queues_task(void*); 
void ble_transmit_packet(ble_packet_t*, uint8_t);
void app_main(void);

// My define's
#define DELAY_MS(x) vTaskDelay((x) / portTICK_PERIOD_MS)
#define SHT3X_INITIAL_CRC 0xff
#define find_median(array, size) find_rank((array), (size), (size)/2)
#define find_min(array, size) find_rank((array), (size), 0)
#define find_max(array, size) find_rank((array), (size), (size)-1)

// I2C related definitions
i2c_master_bus_handle_t bus_handle;
i2c_master_bus_config_t i2c_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = -1,
    .scl_io_num = I2C_SCL_PIN,
    .sda_io_num = I2C_SDA_PIN,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};
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
const int dev_count = sizeof(dev_config)/sizeof(i2c_device_config_t);
i2c_master_dev_handle_t dev_handle[10];

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
    .on_alarm = output,
};

// FreeRTOS handles
QueueHandle_t humidity_queue;
QueueHandle_t temperature_queue;
QueueHandle_t heart_queue;
TaskHandle_t collect_queues_task_handle;
SemaphoreHandle_t max30100_sema_handle;
SemaphoreHandle_t output_sema_handle;

void max30100_irq(void *arg) {
    xSemaphoreGiveFromISR(max30100_sema_handle, NULL);
}

uint16_t bmp180_eeprom[11] = {};
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
        i++;  
    }

    // MAX30100
    command[0] = 0x06; // MODE address
    command[1] = 0x02; // Only heart rate enabled
    i2c_master_transmit(dev_handle[MAX30100], command, 2, -1);

    command[0] = 0x07; // SPO2 configuration
    command[1] = (1 << 1) | (1 << 0); // LED_PW[1:0] = 11 (16 bit resolution)
    i2c_master_transmit(dev_handle[MAX30100], command, 2, -1);

    command[0] = 0x09; // LED configuration address
    command[1] = 0x0f; // maximum current for IR LED
    i2c_master_transmit(dev_handle[MAX30100], command, 2, -1);

    command[0] = 0x01; // Enabling interrupt
    command[1] = (1 << 5); // ENB_HR_RDY
    i2c_master_transmit(dev_handle[MAX30100], command, 2, -1);

    command[0] = 0x00; // Clearing pending interrupt
    i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
    i2c_master_receive(dev_handle[MAX30100], command, 1, -1);
    
    // Hooking interrupt to MAX30100_INT_PIN
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

void sensors_read() {
    uint8_t command[4] = {};
    uint8_t temp[10] = {};
    uint8_t max30100_fifo_wr_ptr = 0; 
    uint8_t max30100_fifo_rd_ptr = 0;

    while (1) {
        // SHT3X
        // Request measurement
        command[0] = 0x24; 
        command[1] = 0x00; 
        i2c_master_transmit(dev_handle[SHT3X], command, 2, -1); 
        DELAY_MS(1);
        // Read humidity
        i2c_master_receive(dev_handle[SHT3X], temp, 6, -1);
        uint16_t sht3x_srh = (((uint16_t)temp[3]) << 8) | temp[4];
        float sht3x_rh = 0;
        // Checksum 
        if (sht3x_crc(sht3x_srh, temp[5] == SHT3X_INITIAL_CRC)) {
            // Relative humiditiy
            sht3x_rh = 100.0 * (float)(sht3x_srh) / (float)((1 << 16) - 1);
        }

        // BMP180 
        // Request measurement
        command[0] = 0xF4;
        command[1] = 0x2E;
        i2c_master_transmit(dev_handle[BMP180], &command[0], 2, -1); 
        DELAY_MS(5);
        // Read temperature
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
                max30100_fifo_wr_ptr += 0x10;
            num_available_samples = max30100_fifo_wr_ptr - max30100_fifo_rd_ptr;

            for (int i = 0; i < num_available_samples; i++) {
                uint8_t temp[4];
                command[0] = 0x05; // FIFO_DATA address
                i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
                for (int j = 0; j < 4; j++) {
                    i2c_master_receive(dev_handle[MAX30100], temp+j, 1, -1);
                }
                float max30100_ir = ((uint16_t)temp[0] << 8) | temp[1];
                xQueueSend(heart_queue, &max30100_ir, portMAX_DELAY); 
            }
        }
        DELAY_MS(1000/SAMPLING_RATE);
    }
}

void swap_f(float *a, float *b) {
    float c = *a;
    *a = *b;
    *b = c;
}

float find_rank(float *array, uint16_t size, uint16_t k) {
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
    float mean = 0;
    float result = 0;
    for (uint16_t i = 0; i < size; i++)
        mean += array[i];
    mean /= (float)size;
    for (uint16_t i = 0; i < size; i++)
        result += powf((array[i] - mean), 2); 
    result /= (float)size;
    result = sqrt(result);
    return result;
}

static bool output(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
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
