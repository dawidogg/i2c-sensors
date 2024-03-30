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
#include "portmacro.h"

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

QueueHandle_t humidity_queue;
QueueHandle_t temperature_queue;
QueueHandle_t heart_queue;
TaskHandle_t sensors_read_task_handle;
SemaphoreHandle_t max30100_sema_handle;

#define SHT3X_INITIAL_CRC 0xff

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

uint8_t intr_count = 0;
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

void sensors_read_task(void *args) {
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
        printf("Read from sht3x: %d %d %d %d\n", temp[0], temp[1], temp[2], temp[3]);
        uint16_t sht3x_srh = (((uint16_t)temp[3]) << 8) | temp[4];
        float sht3x_rh = 0;

        // Checksum 
        //if (sht3x_crc(sht3x_srh, temp[5] == SHT3X_INITIAL_CRC)) {
            // Relative humiditiy
            sht3x_rh = 100.0 * (float)(sht3x_srh) / (float)((1 << 16) - 1);
        //}

        // BMP180 
        command[0] = 0xF4;
        command[1] = 0x2E;
        i2c_master_transmit(dev_handle[BMP180], &command[0], 2, -1); 
        DELAYMS(5);
        command[0] = 0xF6;
        i2c_master_transmit(dev_handle[BMP180], &command[0], 1, -1); 
        i2c_master_receive(dev_handle[BMP180], temp, 3, -1);
        // Uncompensated temperature value
        uint16_t bmp180_ut =  (((uint16_t)temp[0]) << 8) | temp[1];
        float x1 = ((float)(bmp180_ut - bmp180_eeprom[AC6])*bmp180_eeprom[AC5]) / (float)(1 << 15);
        float x2 = (float)(bmp180_eeprom[MC] << 11) / (float)(x1 + bmp180_eeprom[MD]);
        float b5 = x1 + x2;
        // True temperature
        float bmp180_t = (float)(b5 + 8) / (float)(1 << 4) / 10;
    
        if (sht3x_rh != 0)
            xQueueSend(humidity_queue, &sht3x_rh, portMAX_DELAY);
        xQueueSend(temperature_queue, &bmp180_t, portMAX_DELAY);
        printf("Relative humidity: %f%%\n Temperature: %f\n", sht3x_rh, bmp180_t);

        // MAX30100
        if (xSemaphoreTake(max30100_sema_handle, 0) == pdTRUE) {
            command[0] = 0x00;
            i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
            i2c_master_receive(dev_handle[MAX30100], command, 1, -1);
            uint8_t num_available_samples;

            // First transaction: Get the FIFO_WR_PTR:
            command[0] = 0x02; // FIFO_WR_PTR address
            i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
            i2c_master_receive(dev_handle[MAX30100], &max30100_fifo_wr_ptr, 1, -1);

            // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
            if (max30100_fifo_wr_ptr <= max30100_fifo_rd_ptr)
                max30100_fifo_wr_ptr += 0x10;
            num_available_samples = max30100_fifo_wr_ptr - max30100_fifo_rd_ptr;

            command[0] = 0x05; // FIFO_DATA address
            for (int i = 0; i < num_available_samples; i++) {
                uint8_t temp[4];
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

            // Third transaction: Write to FIFO_RD_PTR register.
            command[0] = 0x04; // FIFO_RD_PTR address
            i2c_master_transmit(dev_handle[MAX30100], command, 1, -1);
            i2c_master_receive(dev_handle[MAX30100], &max30100_fifo_rd_ptr, 1, -1);
        }
        DELAYMS(500);
    }
}


void app_main(void) {
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus_handle));

    for (int i = 0; i < dev_count; i++) {
        if (i2c_master_probe(bus_handle, dev_config[i].device_address, -1) == ESP_OK) {
            printf("Sensor no %d is available\n", i);
            ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config[i], &dev_handle[i]));
        } else {
            printf("Application failed to start, connect all the sensors\n");
            return;
        }
    }
    
    humidity_queue = xQueueCreate(256, sizeof(float));
    temperature_queue = xQueueCreate(256, sizeof(float));
    heart_queue = xQueueCreate(256, sizeof(float));
    max30100_sema_handle = xSemaphoreCreateBinary();

    sensors_configure();
    xTaskCreate(sensors_read_task, "SENSORS_TASK", 4096, NULL, 0, &sensors_read_task_handle);
}

// idf.py -B build.clang -D IDF_TOOLCHAIN=clang reconfigure
