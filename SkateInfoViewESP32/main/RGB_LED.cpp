#include "RGB_LED.h"
#include <math.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// LEDC PWM output pin
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R          LEDC_CHANNEL_0
#define LEDC_CHANNEL_G          LEDC_CHANNEL_1
#define LEDC_CHANNEL_B          LEDC_CHANNEL_2
#define LEDC_DUTY_RES           LEDC_TIMER_8_BIT
#define LEDC_CLK_SRC            LEDC_AUTO_CLK
#define LEDC_FREQUENCY          (4000) // Frequency in Hertz. Set frequency at 4 kHz

void configurePWMPin(int pinNumber, ledc_channel_t channel){
    static ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER_1,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = pinNumber,
        .speed_mode     = LEDC_MODE,
        .channel        = channel,
        .timer_sel      = ledc_timer.timer_num,
        .duty           = 0,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

ESP32LED::ESP32LED(int pinR, int pinG, int pinB)
    : rPin(pinR), gPin(pinG), bPin(pinB),
      currentPattern(LEDPattern::Solid),
      currentColor({0,0,0}),
      currentSpeed(PatternSpeed::Medium) {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_CLK_SRC,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_R = {
        .gpio_num       = pinR,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_R,
        .timer_sel      = LEDC_TIMER,
        .duty           = 255,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_R));

    ledc_channel_config_t ledc_channel_G = {
        .gpio_num       = pinG,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_G,
        .timer_sel      = LEDC_TIMER,
        .duty           = 255,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_G));

    ledc_channel_config_t ledc_channel_B = {
        .gpio_num       = pinB,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_B,
        .timer_sel      = LEDC_TIMER,
        .duty           = 255,
        .hpoint         = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_B));
    // Add similar configs for G and B if needed

    xTaskCreate(taskFunc, "LEDTask", 2048, this, 1, &taskHandle);
}

void ESP32LED::color(const RGBColor& c) {
    currentColor = c;
    currentPattern = LEDPattern::Solid;
}

void ESP32LED::color(uint8_t R, uint8_t G, uint8_t B) {
    currentColor.r = R;
    currentColor.g = G;
    currentColor.b = B;
    currentPattern = LEDPattern::Solid;
}

void ESP32LED::control(bool enable) {
    if (!enable) {
        setChannel(LEDC_CHANNEL_R, 0);
        setChannel(LEDC_CHANNEL_G, 0);
        setChannel(LEDC_CHANNEL_B, 0);
    }
}

void ESP32LED::reset() {
    control(false);
}

void ESP32LED::setPattern(LEDPattern pattern, const RGBColor& c, PatternSpeed speed) {
    currentPattern = pattern;
    currentColor = c;
    currentSpeed = speed;
}

void ESP32LED::taskFunc(void* arg) {
    ESP32LED* self = static_cast<ESP32LED*>(arg);
    for (;;) {
        self->updatePattern();
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}

void ESP32LED::updatePattern() {
    switch (currentPattern) {
        case LEDPattern::Solid:
            setChannel(LEDC_CHANNEL_R, currentColor.r);
            setChannel(LEDC_CHANNEL_G, currentColor.g);
            setChannel(LEDC_CHANNEL_B, currentColor.b);
            break;
        case LEDPattern::Blink: {
            static bool on = false;
            on = !on;
            if (on) {
                setChannel(LEDC_CHANNEL_R, currentColor.r);
                setChannel(LEDC_CHANNEL_G, currentColor.g);
                setChannel(LEDC_CHANNEL_B, currentColor.b);
            } else {
                setChannel(LEDC_CHANNEL_R, 0);
                setChannel(LEDC_CHANNEL_G, 0);
                setChannel(LEDC_CHANNEL_B, 0);
            }
            break;
        }
        case LEDPattern::Breathe: {
            static int brightness = 0;
            static int step = 1; // +1 or -1

            brightness += step;

            if (brightness >= 255 || brightness <= 0) {
                step = -step; // reverse direction
            }
            setChannel(LEDC_CHANNEL_R, (currentColor.r * brightness) / 255);
            setChannel(LEDC_CHANNEL_G, (currentColor.g * brightness) / 255);
            setChannel(LEDC_CHANNEL_B, (currentColor.b * brightness) / 255);
            break;
        }
        default:
            break;
    }
}

void ESP32LED::setChannel(ledc_channel_t channel, uint8_t value) {
    // Map 0–255 to PWM duty cycle
    // Example: ledcWrite(channel, value);
    // Set duty to value
    // You may need to map pin to channel here if using multiple channels
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, 255-value));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
}
