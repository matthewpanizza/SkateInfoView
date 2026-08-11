# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Enable loading the ESP-IDF component from the Pizzatronix ESP32 submodule.
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/main/Pizzatronix_ESP32/pizzatronix_esp32)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(SkateInfoViewESP32)
