#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "portmacro.h"
#include "driver/i2c_master.h"

#define LED_PIN 23

const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

gpio_config_t led_pin_config = {
  (uint64_t)  (1UL << LED_PIN),
  (gpio_mode_t) GPIO_MODE_OUTPUT, // mode
  (gpio_pullup_t) GPIO_PULLUP_DISABLE, // pull_up_en
  (gpio_pulldown_t) GPIO_PULLDOWN_DISABLE, // pull_down_en
  (gpio_int_type_t) GPIO_INTR_DISABLE // intr_type
};

void app_main(void) {
    gpio_config(&led_pin_config);
    
    while (1) { 
        vTaskDelay(xDelay);
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(xDelay);
        gpio_set_level(LED_PIN, 0);
    }

}

// idf.py -B build.clang -D IDF_TOOLCHAIN=clang reconfigure
