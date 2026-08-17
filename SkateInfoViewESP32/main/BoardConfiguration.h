#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

class INVSStorage;

struct MotorSenseConfiguration {
    uint8_t motorEnableMask;
    uint16_t motor1PulsesPerRevolution;
    uint16_t motor2PulsesPerRevolution;
};

struct BoardConfiguration {
    uint16_t structRevision;
    uint16_t hardwareRevision;
    MotorSenseConfiguration motorSenseConfiguration;
    uint16_t wheelSizeMillimeterTenths;
    uint8_t padding[128 - sizeof(uint16_t) - sizeof(uint16_t)
                    - sizeof(MotorSenseConfiguration) - sizeof(uint16_t) - sizeof(uint32_t)];
    uint32_t crc32;
};

static_assert(sizeof(MotorSenseConfiguration) == 6, "MotorSenseConfiguration must be 6 bytes");
static_assert(sizeof(BoardConfiguration) == 128, "BoardConfiguration must be 128 bytes");

constexpr const char* BOARD_CONFIGURATION_NVS_KEY = "board_config";

void board_configuration_set_defaults(BoardConfiguration& configuration);
bool board_configuration_is_valid(const BoardConfiguration& configuration,
                                  INVSStorage& storage);
esp_err_t board_configuration_load(INVSStorage& storage,
                                   BoardConfiguration& configuration);
esp_err_t board_configuration_save(INVSStorage& storage,
                                   BoardConfiguration& configuration);