#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 23
#define MAX30100_INT_PIN 19
#define SAMPLING_RATE 1 // Hz 
#define BLE_OUTPUT_RATE 1 // seconds

#define DELAY_MS(x) vTaskDelay((x) / portTICK_PERIOD_MS)
