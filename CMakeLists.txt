# The following four lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Exclude unused ESP-IDF components to reduce binary size
set(EXCLUDE_COMPONENTS
    esp_https_ota
    esp_http_server
    fatfs
    spiffs
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(ledrx)
