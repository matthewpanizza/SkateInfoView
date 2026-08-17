#include "BoardConfiguration.h"

#include <string.h>

#include "INVSStorage.h"

namespace {
constexpr uint16_t BOARD_CONFIG_STRUCT_REVISION = 1;
constexpr size_t BOARD_CONFIGURATION_CRC_OFFSET = offsetof(BoardConfiguration, crc32);
}

void board_configuration_set_defaults(BoardConfiguration& configuration)
{
    memset(&configuration, 0, sizeof(configuration));
    configuration.structRevision = BOARD_CONFIG_STRUCT_REVISION;
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