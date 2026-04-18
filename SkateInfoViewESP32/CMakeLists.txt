# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Enable loading custom components from main/Pizzatronix_Common/ESP32 (submodule)
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/main/Pizzatronix_Common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(SkateInfoViewESP32)
