#ifndef RGB_LED_H
#define RGB_LED_H

#include "SkateInfoViewCommon/ILED.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>

void configurePWMPin(int pinNumber, ledc_channel_t channel);

class ESP32LED : public ILED {
public:
    ESP32LED(int pinR, int pinG, int pinB);
    void color(const RGBColor& c) override;
    void color(uint8_t R, uint8_t G, uint8_t B) override;
    void control(bool enable) override;
    void reset() override;
    void setPattern(LEDPattern pattern, const RGBColor& c, PatternSpeed speed) override;
private:
    int rPin, gPin, bPin;
    TaskHandle_t taskHandle;
    LEDPattern currentPattern;
    RGBColor currentColor;
    PatternSpeed currentSpeed;
    static void taskFunc(void* arg);
    void updatePattern();
    void setChannel(ledc_channel_t channel, uint8_t value);
};

#endif // RGB_LED_H
