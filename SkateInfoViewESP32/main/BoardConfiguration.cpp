#include "BoardConfiguration.h"

#include <string.h>

#include "INVSStorage.h"
#include "MenuItem.h"
#include "ISerialInterfaceCombiner.h"

#include <cstdlib>
#include <cstring>

namespace {
constexpr uint16_t BOARD_CONFIG_STRUCT_REVISION = 1;
constexpr size_t BOARD_CONFIGURATION_CRC_OFFSET = offsetof(BoardConfiguration, crc32);

bool parse_uint8(const std::string& command, const char* name, uint8_t& value)
{
    const size_t name_length = strlen(name);
    if (command.compare(0, name_length, name) != 0 ||
        (command.size() > name_length && command[name_length] != ' ')) {
        return false;
    }

    char* end = nullptr;
    const unsigned long parsed = strtoul(command.c_str() + name_length, &end, 0);
    if (end == command.c_str() + name_length || *end != '\0' || parsed > UINT8_MAX) {
        return false;
    }

    value = static_cast<uint8_t>(parsed);
    return true;
}

bool parse_uint16(const std::string& command, const char* name, uint16_t& value)
{
    const size_t name_length = strlen(name);
    if (command.compare(0, name_length, name) != 0 ||
        (command.size() > name_length && command[name_length] != ' ')) {
        return false;
    }

    char* end = nullptr;
    const unsigned long parsed = strtoul(command.c_str() + name_length, &end, 0);
    if (end == command.c_str() + name_length || *end != '\0' || parsed > UINT16_MAX) {
        return false;
    }

    value = static_cast<uint16_t>(parsed);
    return true;
}

void mark_board_configuration_stale()
{
    __atomic_store_n(&board_configuration_stale, true, __ATOMIC_RELEASE);
}

void print_board_configuration_help()
{
    console_combiner.writeLine("Board configuration commands:");
    console_combiner.writeLine("  hardware <value>   Set hardware revision");
    console_combiner.writeLine("  motors <value>     Set motor enable mask");
    console_combiner.writeLine("  motor1 <value>     Set motor 1 pulses per revolution");
    console_combiner.writeLine("  motor2 <value>     Set motor 2 pulses per revolution");
    console_combiner.writeLine("  diameter <value>   Set wheel diameter in 0.1 mm (1050 = 105.0 mm)");
    console_combiner.writeLine("  uart menu|openocd  Set USB Serial/JTAG mode");
    console_combiner.writeLine("Current values:");
    console_combiner.writeLine("  hardware=" + std::to_string(boardConfiguration.hardwareRevision));
    console_combiner.writeLine("  motors=" + std::to_string(boardConfiguration.motorSenseConfiguration.motorEnableMask));
    console_combiner.writeLine("  motor1=" + std::to_string(boardConfiguration.motorSenseConfiguration.motor1PulsesPerRevolution));
    console_combiner.writeLine("  motor2=" + std::to_string(boardConfiguration.motorSenseConfiguration.motor2PulsesPerRevolution));
    console_combiner.writeLine("  diameter=" + std::to_string(boardConfiguration.wheelSizeMillimeterTenths));
    console_combiner.writeLine(std::string("  uart=") +
                               (boardConfiguration.uartConfig == STANDARD_MENU ? "menu" : "openocd"));
}

void board_configuration_menu_on_enter()
{
    print_board_configuration_help();
}
}

BoardConfiguration boardConfiguration{};
volatile bool board_configuration_stale = false;
MenuItem board_configuration_menu("board", board_configuration_menu_on_enter);

void board_configuration_menu_initialize()
{
    static bool initialized = false;
    if (initialized) {
        return;
    }
    initialized = true;

    board_configuration_menu.addCommand("h", nullptr, [](std::string*) {
        print_board_configuration_help();
    });
    board_configuration_menu.addCommand("help", nullptr, [](std::string*) {
        print_board_configuration_help();
    });
    board_configuration_menu.addCommand("hardware", nullptr, [](std::string* command) {
        uint16_t value = 0;
        if (*command == "hardware") {
            console_combiner.writeLine("Usage: hardware <value> (current: " +
                                       std::to_string(boardConfiguration.hardwareRevision) + ")");
        } else if (parse_uint16(*command, "hardware", value)) {
            boardConfiguration.hardwareRevision = value;
            mark_board_configuration_stale();
            console_combiner.writeLine("Hardware revision set to " + std::to_string(value));
        }
    });
    board_configuration_menu.addCommand("motors", nullptr, [](std::string* command) {
        uint8_t value = 0;
        if (*command == "motors") {
            console_combiner.writeLine("Usage: motors <value> (current: " +
                                       std::to_string(boardConfiguration.motorSenseConfiguration.motorEnableMask) + ")");
        } else if (parse_uint8(*command, "motors", value)) {
            boardConfiguration.motorSenseConfiguration.motorEnableMask = value;
            mark_board_configuration_stale();
            console_combiner.writeLine("Motor enable mask set to " + std::to_string(value));
        }
    });
    board_configuration_menu.addCommand("motor1", nullptr, [](std::string* command) {
        uint16_t value = 0;
        if (*command == "motor1") {
            console_combiner.writeLine("Usage: motor1 <value> (current: " +
                                       std::to_string(boardConfiguration.motorSenseConfiguration.motor1PulsesPerRevolution) + ")");
        } else if (parse_uint16(*command, "motor1", value)) {
            boardConfiguration.motorSenseConfiguration.motor1PulsesPerRevolution = value;
            mark_board_configuration_stale();
            console_combiner.writeLine("Motor 1 pulses per revolution set to " + std::to_string(value));
        }
    });
    board_configuration_menu.addCommand("motor2", nullptr, [](std::string* command) {
        uint16_t value = 0;
        if (*command == "motor2") {
            console_combiner.writeLine("Usage: motor2 <value> (current: " +
                                       std::to_string(boardConfiguration.motorSenseConfiguration.motor2PulsesPerRevolution) + ")");
        } else if (parse_uint16(*command, "motor2", value)) {
            boardConfiguration.motorSenseConfiguration.motor2PulsesPerRevolution = value;
            mark_board_configuration_stale();
            console_combiner.writeLine("Motor 2 pulses per revolution set to " + std::to_string(value));
        }
    });
    board_configuration_menu.addCommand("diameter", nullptr, [](std::string* command) {
        uint16_t value = 0;
        if (*command == "diameter") {
            console_combiner.writeLine("Usage: diameter <tenths_mm> (current: " +
                                       std::to_string(boardConfiguration.wheelSizeMillimeterTenths) + ")");
        } else if (parse_uint16(*command, "diameter", value)) {
            boardConfiguration.wheelSizeMillimeterTenths = value;
            mark_board_configuration_stale();
            console_combiner.writeLine("Wheel diameter set to " + std::to_string(value) + " tenths of a millimeter");
        }
    });
    board_configuration_menu.addCommand("uart", nullptr, [](std::string* command) {
        if (*command == "uart") {
            console_combiner.writeLine("Usage: uart menu|openocd (current: " +
                                       std::string(boardConfiguration.uartConfig == STANDARD_MENU ? "menu" : "openocd") + ")");
        } else if (*command == "uart menu") {
            boardConfiguration.uartConfig = STANDARD_MENU;
            mark_board_configuration_stale();
            console_combiner.writeLine("UART mode set to menu");
        } else if (*command == "uart openocd") {
            boardConfiguration.uartConfig = STANDARD_ESPLOG;
            mark_board_configuration_stale();
            console_combiner.writeLine("UART mode set to openocd");
        }
    });
}

void board_configuration_set_defaults(BoardConfiguration& configuration)
{
    memset(&configuration, 0, sizeof(configuration));
    configuration.structRevision = BOARD_CONFIG_STRUCT_REVISION;
    configuration.motorSenseConfiguration.motorEnableMask = 0x3F;
    configuration.motorSenseConfiguration.motor1PulsesPerRevolution = 44;
    configuration.motorSenseConfiguration.motor2PulsesPerRevolution = 44;
    configuration.wheelSizeMillimeterTenths = 1050;
}

bool board_configuration_is_valid(const BoardConfiguration& configuration,
                                  INVSStorage& storage)
{
    return configuration.structRevision == BOARD_CONFIG_STRUCT_REVISION
        && configuration.crc32 == storage.crc32_compute(&configuration, BOARD_CONFIGURATION_CRC_OFFSET);
}

esp_err_t board_configuration_load(INVSStorage& storage,
                                   BoardConfiguration& configuration)
{
    if (!storage.key_exists(BOARD_CONFIGURATION_NVS_KEY)) {
        board_configuration_set_defaults(configuration);
        return board_configuration_save(storage, configuration);
    }

    esp_err_t err = storage.load(BOARD_CONFIGURATION_NVS_KEY, configuration);
    if (err != ESP_OK || !board_configuration_is_valid(configuration, storage)) {
        board_configuration_set_defaults(configuration);
        return board_configuration_save(storage, configuration);
    }

    return ESP_OK;
}

esp_err_t board_configuration_save(INVSStorage& storage,
                                   BoardConfiguration& configuration)
{
    configuration.crc32 = storage.crc32_compute(&configuration, BOARD_CONFIGURATION_CRC_OFFSET);
    return storage.save(BOARD_CONFIGURATION_NVS_KEY, configuration);
}