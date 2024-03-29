#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "hal/i2c_types.h"

#define DELAYMS(x) vTaskDelay((x) / portTICK_PERIOD_MS)
#define I2C_SCL 22
#define I2C_SDA 23

i2c_master_bus_config_t i2c_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = -1,
    .scl_io_num = I2C_SCL,
    .sda_io_num = I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};
i2c_master_bus_handle_t bus_handle;

typedef enum {
    TEMPHUM,
    PRESSURE,
    HEART,
} SENSOR_ID;

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

void app_main(void) {
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_config, &bus_handle));

    for (int i = 0; i < dev_count; i++) {
        if (i2c_master_probe(bus_handle, dev_config[i].device_address, -1) == ESP_OK) {
            printf("Sensor no %d is available\n", i);
            ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config[i], &dev_handle[i]));
        }
    }

    uint8_t command[2];
    uint8_t heart_fifo_wr_ptr = 0; 
    uint8_t heart_fifo_rd_ptr = 0;
    uint8_t num_available_samples;
    uint16_t heart_ir, heart_red;
    // Heart rate test
    // Configuration

    command[0] = 0x06; // FIFO_WR_PTR address
    command[1] = 0x02; // FIFO_WR_PTR address
    i2c_master_transmit(dev_handle[HEART], command, 2, -1);

    while (1) {
        DELAYMS(15);
        // First transaction: Get the FIFO_WR_PTR:
        command[0] = 0x02; // FIFO_WR_PTR address
        i2c_master_transmit(dev_handle[HEART], command, 1, -1);
        i2c_master_receive(dev_handle[HEART], &heart_fifo_wr_ptr, 1, -1);
        printf("heart_fifo_wr_ptr: %x\n", heart_fifo_wr_ptr);

        // Second transaction: Read NUM_SAMPLES_TO_READ samples from the FIFO
        if (heart_fifo_wr_ptr < heart_fifo_rd_ptr)
            heart_fifo_wr_ptr += 0x10;
        num_available_samples = heart_fifo_wr_ptr - heart_fifo_rd_ptr;

        command[0] = 0x05; // FIFO_DATA address
        i2c_master_transmit(dev_handle[HEART], command, 1, -1);
        for (int i = 0; i < num_available_samples; i++) {
            uint8_t temp[4];
            i2c_master_receive(dev_handle[HEART], temp, 4, -1);
            heart_ir = (temp[0] << 8) | temp[1];
            heart_red = (temp[2] << 8) | temp[3];
            printf("IR: %x RED: %x\n", heart_ir, heart_red);
        }

        // Third transaction: Write to FIFO_RD_PTR register.
        command[0] = 0x04; // FIFO_RD_PTR address
        i2c_master_transmit(dev_handle[HEART], command, 1, -1);
        i2c_master_receive(dev_handle[HEART], &heart_fifo_rd_ptr, 1, -1);
        printf("heart_fifo_rd_ptr: %x\n", heart_fifo_rd_ptr);
    }
    while (1);

    // Pressure sensor
    uint8_t pressure_buffer[10] = {};

    printf("Printing registers\n");
    for (command[0] = 0xAA; command[0] < 0xBF; command[0] += 2) {
        i2c_master_transmit(dev_handle[PRESSURE], command, 1, -1); 
        DELAYMS(10);
        i2c_master_receive(dev_handle[PRESSURE], pressure_buffer, 2, -1);
        printf("%d %d\n", pressure_buffer[0], pressure_buffer[1]);
    }

    printf("Printing temperature\n");
    while (1) {
        command[0] = 0xF4;
        command[1] = 0x2E;
        i2c_master_transmit(dev_handle[PRESSURE], &command[0], 2, -1); 
        DELAYMS(5);
        command[0] = 0xF6;
        i2c_master_transmit(dev_handle[PRESSURE], &command[0], 1, -1); 
        i2c_master_receive(dev_handle[PRESSURE], pressure_buffer, 3, -1);
        printf("%d %d\n", pressure_buffer[0], pressure_buffer[1]);
        DELAYMS(50);
    }

    // TEMPHUM test
    command[0] = 0x24; 
    command[1] = 0x00; 
    uint8_t temphum_buffer[10] = {};
    while (1) {
        DELAYMS(50);
        i2c_master_transmit(dev_handle[TEMPHUM], command, 2, -1); 
        i2c_master_receive(dev_handle[TEMPHUM], temphum_buffer, 6, -1);

        printf("Data read: ");
        for (int i = 0; i < 6; i++)
            printf("%d ", temphum_buffer[i]);
        printf("\n");
    }

    for (int i = 0; i < dev_count; i++)
        i2c_master_bus_rm_device(dev_handle[i]);
}

// idf.py -B build.clang -D IDF_TOOLCHAIN=clang reconfigure
