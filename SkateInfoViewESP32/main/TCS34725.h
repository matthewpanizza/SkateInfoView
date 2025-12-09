#ifndef TCS34725_H
#define TCS34725_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

// TCS34725 I2C Address
#define TCS34725_I2C_ADDR 0x29

// Register definitions
#define TCS34725_ENABLE     0x00
#define TCS34725_ATIME      0x01
#define TCS34725_WTIME      0x03
#define TCS34725_AILTL      0x04
#define TCS34725_AILTH      0x05
#define TCS34725_AIHTL      0x06
#define TCS34725_AIHTH      0x07
#define TCS34725_PERS       0x0C
#define TCS34725_CONFIG     0x0D
#define TCS34725_CONTROL    0x0F
#define TCS34725_ID         0x12
#define TCS34725_STATUS     0x13
#define TCS34725_CDATA      0x14
#define TCS34725_CDATAH     0x15
#define TCS34725_RDATA      0x16
#define TCS34725_RDATAH     0x17
#define TCS34725_GDATA      0x18
#define TCS34725_GDATAH     0x19
#define TCS34725_BDATA      0x1A
#define TCS34725_BDATAH     0x1B

// Enable register bits
#define TCS34725_ENABLE_PON 0x01
#define TCS34725_ENABLE_AEN 0x02

// ATIME values (higher = longer integration time, more sensitivity)
#define TCS34725_ATIME_2_4MS   0xFF
#define TCS34725_ATIME_24MS    0xF6
#define TCS34725_ATIME_101MS   0xD5
#define TCS34725_ATIME_154MS   0xC0
#define TCS34725_ATIME_700MS   0x00

// Gain values
#define TCS34725_GAIN_1X  0x00
#define TCS34725_GAIN_4X  0x01
#define TCS34725_GAIN_16X 0x02
#define TCS34725_GAIN_60X 0x03

// Color data structure
typedef struct {
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} RGBWColor;

class TCS34725 {
public:
    TCS34725(gpio_num_t sclPin, gpio_num_t sdaPin);
    ~TCS34725();
    
    esp_err_t init();
    esp_err_t setIntegrationTime(uint8_t atime);
    esp_err_t setGain(uint8_t gain);
    esp_err_t enable();
    esp_err_t disable();
    esp_err_t getRawColor(RGBWColor* color);
    uint8_t getDeviceID();
    
private:
    gpio_num_t sclPin, sdaPin;
    i2c_master_bus_handle_t i2cBus;
    i2c_master_dev_handle_t i2cDev;
    
    esp_err_t writeRegister(uint8_t reg, uint8_t value);
    esp_err_t readRegister(uint8_t reg, uint8_t* value);
    esp_err_t readTwoBytes(uint8_t reg, uint16_t* value);
};

#endif // TCS34725_H
