#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "adxl.h"

static const char *TAG = "main";

void app_main(void)
{
    uint8_t data[1];
    uint8_t data_buffer[6];
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Read the adxl345 WHO_AM_I register, on power up the register should have the value 0x53
    ESP_ERROR_CHECK(adxl345_register_read(dev_handle, ADXL345_DEVID_REG_ADDR, data, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);

    ESP_LOGI(TAG, "Enabling measurement mode");
    ESP_ERROR_CHECK(adxl345_register_write_byte(dev_handle, ADXL345_POWER_CTL_REG_ADDR, 1 << ADXL345_MEASURE_BIT));

    ESP_LOGI(TAG, "Reading 6 bytes of axis data...");
    while (1) {
        // 1. Attempt to read the data and capture the return status
        esp_err_t err = adxl345_register_read(dev_handle, ADXL345_DATAX0_REG_ADDR, data_buffer, 6);

        // 2. Check if the read was successful (err == ESP_OK)
        if (err == ESP_OK) {
            int16_t x_axis = (data_buffer[1] << 8) | data_buffer[0];
            int16_t y_axis = (data_buffer[3] << 8) | data_buffer[2];
            int16_t z_axis = (data_buffer[5] << 8) | data_buffer[4];

            ESP_LOGI(TAG, "X=%d, Y=%d, Z=%d", x_axis, y_axis, z_axis);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}