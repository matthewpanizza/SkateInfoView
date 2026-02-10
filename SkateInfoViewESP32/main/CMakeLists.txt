idf_component_register(SRCS "mcp_can.cpp" "TCS34725.cpp" "RGB_LED.cpp" "SkateInfoViewESP32.cpp"
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver nvs_flash esp_adc freertos bt
                    INCLUDE_DIRS ".")
