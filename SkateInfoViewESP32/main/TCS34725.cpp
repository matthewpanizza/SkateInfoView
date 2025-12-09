#include "TCS34725.h"
#include <esp_err.h>
#include <esp_log.h>
#include <esp_check.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "TCS34725";

TCS34725::TCS34725(gpio_num_t sclPin, gpio_num_t sdaPin)
    : sclPin(sclPin), sdaPin(sdaPin), i2cBus(nullptr), i2cDev(nullptr) {}

TCS34725::~TCS34725() {
    if (i2cDev) {
        i2c_master_bus_rm_device(i2cDev);
    }
    if (i2cBus) {
        i2c_del_master_bus(i2cBus);
    }
}

esp_err_t TCS34725::init() {
    // Initialize I2C master bus
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = sdaPin,
        .scl_io_num = sclPin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&i2c_mst_config, &i2cBus), TAG, "Failed to create I2C bus");

    // Add TCS34725 device to bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = TCS34725_I2C_ADDR,
        .scl_speed_hz = 100000,  // 100 kHz
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2cBus, &dev_cfg, &i2cDev), TAG, "Failed to add device to bus");

    // Verify device ID
    uint8_t deviceId = getDeviceID();
    if (deviceId != 0x44 && deviceId != 0x4D) {
        ESP_LOGE(TAG, "Unknown device ID: 0x%02X (expected 0x44 or 0x4D)", deviceId);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "TCS34725 found with ID 0x%02X", deviceId);

    // Enable the sensor (power on + AEN)
    ESP_RETURN_ON_ERROR(writeRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON), TAG, "Failed to enable power");
    
    // Wait for power to stabilize
    vTaskDelay(pdMS_TO_TICKS(3));
    
    // Set integration time
    ESP_RETURN_ON_ERROR(setIntegrationTime(TCS34725_ATIME_101MS), TAG, "Failed to set integration time");
    
    // Set gain
    ESP_RETURN_ON_ERROR(setGain(TCS34725_GAIN_1X), TAG, "Failed to set gain");
    
    // Enable color sensing
    ESP_RETURN_ON_ERROR(writeRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN), TAG, "Failed to enable AEN");
    
    ESP_LOGI(TAG, "TCS34725 initialized successfully");
    return ESP_OK;
}

esp_err_t TCS34725::setIntegrationTime(uint8_t atime) {
    return writeRegister(TCS34725_ATIME, atime);
}

esp_err_t TCS34725::setGain(uint8_t gain) {
    return writeRegister(TCS34725_CONTROL, gain & 0x03);
}

esp_err_t TCS34725::enable() {
    return writeRegister(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
}

esp_err_t TCS34725::disable() {
    return writeRegister(TCS34725_ENABLE, 0x00);
}

esp_err_t TCS34725::getRawColor(RGBWColor* color) {
    if (!color) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_RETURN_ON_ERROR(readTwoBytes(TCS34725_CDATA, &color->clear), TAG, "Failed to read clear data");
    ESP_RETURN_ON_ERROR(readTwoBytes(TCS34725_RDATA, &color->red), TAG, "Failed to read red data");
    ESP_RETURN_ON_ERROR(readTwoBytes(TCS34725_GDATA, &color->green), TAG, "Failed to read green data");
    ESP_RETURN_ON_ERROR(readTwoBytes(TCS34725_BDATA, &color->blue), TAG, "Failed to read blue data");
    
    return ESP_OK;
}

uint8_t TCS34725::getDeviceID() {
    uint8_t id = 0;
    readRegister(TCS34725_ID, &id);
    return id;
}

esp_err_t TCS34725::writeRegister(uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {(uint8_t)(reg | 0x80), value};  // Set command bit (0x80)
    return i2c_master_transmit(i2cDev, write_buf, sizeof(write_buf), -1);
}

esp_err_t TCS34725::readRegister(uint8_t reg, uint8_t* value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t cmd = reg | 0x80;  // Set command bit
    return i2c_master_transmit_receive(i2cDev, &cmd, 1, value, 1, -1);
}

esp_err_t TCS34725::readTwoBytes(uint8_t reg, uint16_t* value) {
    if (!value) {
        return ESP_ERR_INVALID_ARG;
    }
    uint8_t cmd = reg | 0x80;
    uint8_t buf[2];
    ESP_RETURN_ON_ERROR(i2c_master_transmit_receive(i2cDev, &cmd, 1, buf, 2, -1), TAG, "Failed to read bytes");
    *value = (buf[1] << 8) | buf[0];  // Little-endian
    return ESP_OK;
}
