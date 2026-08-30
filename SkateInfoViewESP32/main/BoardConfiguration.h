#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "UartConfig.h"

class INVSStorage;
class MenuItem;
class ISerialInterfaceCombiner;

struct MotorSenseConfiguration {
    uint8_t motorEnableMask;                //Chanels: b0 = A1, b1 = A2, b2 = A3, b3 = B1, b4 = B2, b5 = B3
    uint16_t motor1PulsesPerRevolution;
    uint16_t motor2PulsesPerRevolution;
};

struct BoardConfiguration {
    uint16_t structRevision;
    uint16_t hardwareRevision;
    MotorSenseConfiguration motorSenseConfiguration;
    uint16_t wheelSizeMillimeterTenths;
    UartConfigToggle uartConfig;
    uint8_t padding[128 - sizeof(uint16_t) - sizeof(uint16_t)
                    - sizeof(MotorSenseConfiguration) - sizeof(uint16_t) - sizeof(UartConfigToggle) - sizeof(uint32_t)];
    uint32_t crc32;
};

extern BoardConfiguration boardConfiguration;
extern volatile bool board_configuration_stale;
extern MenuItem board_configuration_menu;
extern ISerialInterfaceCombiner console_combiner;

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

void board_configuration_menu_initialize();