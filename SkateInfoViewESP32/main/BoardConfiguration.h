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

struct PowerSystemConfiguration {
    uint16_t voltage_12V_millivolts;
    uint16_t voltage_5V_millivolts;
    uint16_t adc_counts_per_amp_12V;
    uint16_t adc_counts_per_amp_5V;
};

struct HardwareFeatureEnable {
    bool feature0 : 1;
    bool feature1 : 1;
    bool feature2 : 1;
    bool feature3 : 1;
    bool feature4 : 1;
    bool feature5 : 1;
    bool feature6 : 1;
    bool feature7 : 1;
    bool feature8 : 1;
    bool feature9 : 1;
    bool feature10 : 1;
    bool feature11 : 1;
    bool feature12 : 1;
    bool feature13 : 1;
    bool feature14 : 1;
    bool tmp117SensorEnabled : 1;
    bool canbusEnabled : 1;
    bool useMCP2515 : 1;
    bool motorSenseEnable : 1;
};

struct BoardConfiguration {
    uint16_t structRevision;
    uint16_t hardwareRevision;
    MotorSenseConfiguration motorSenseConfiguration;
    uint16_t wheelSizeMillimeterTenths;
    UartConfigToggle uartConfig;
    PowerSystemConfiguration powerSystemConfiguration;
    HardwareFeatureEnable hardwareFeatureEnable;
    uint8_t padding[128 - sizeof(uint16_t) - sizeof(uint16_t)
                    - sizeof(MotorSenseConfiguration) - sizeof(uint16_t)
                    - sizeof(UartConfigToggle) - sizeof(PowerSystemConfiguration)
                    - sizeof(HardwareFeatureEnable) - sizeof(uint32_t) - 4];
    uint32_t crc32;
};

extern BoardConfiguration boardConfiguration;
extern volatile bool board_configuration_stale;
extern MenuItem board_configuration_menu;
extern ISerialInterfaceCombiner console_combiner;

static_assert(sizeof(MotorSenseConfiguration) == 6, "MotorSenseConfiguration must be 6 bytes");
static_assert(sizeof(PowerSystemConfiguration) == 8, "PowerSystemConfiguration must be 8 bytes");
static_assert(sizeof(HardwareFeatureEnable) == 3, "HardwareFeatureEnable must be 3 bytes");
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