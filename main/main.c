#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "i2c_commands.h"

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
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x44,
    .scl_speed_hz = 100000,
    },
    {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x77,
    .scl_speed_hz = 100000,
    },
    {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
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

    // TEMPHUM test
    uint8_t command[2];
    command[0] = CMD_TEMPHUM_SINGLESHOT >> 8; 
    command[1] = CMD_TEMPHUM_SINGLESHOT & 0xFF; 
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

    // uint8_t read_buffer[1024];
    // while (1) {
    //     for (int i = 0; i < dev_count; i++) {
    //         i2c_master_receive(dev_handle[i], read_buffer, 1, -1);  
    //         printf("%d\n", read_buffer[i]);
    //     }
    // } 
}

// idf.py -B build.clang -D IDF_TOOLCHAIN=clang reconfigure
