idf_component_register(SRCS "SkateInfoViewESP32.cpp"
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver nvs_flash esp_adc freertos bt
                    INCLUDE_DIRS ".")
