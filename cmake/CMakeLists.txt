# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

add_definitions(
  # this option should be used, but OpenMRN does not compile
  # unfortunately all the network functions are non-blocking/polling when
  # using the arduino interface
  #-D__FreeRTOS__            
  -DESP32=1 
  -DARDUINO_ESP32_DEV 
  -DARDUINO=1 
  -DCONFIG_AUTOSTART_ARDUINO=1 
  -DCORE_DEBUG_LEVEL=ARDUHAL_LOG_LEVEL_INFO -g3
  -DMBEDTLS_KEY_EXCHANGE_PSK_ENABLED
)

# espressif IDF has the 'git_submodule_check' function which we want to use,
# include the git_submodules cmake module:
set(IDF_PATH $ENV{IDF_PATH})
set(CMAKE_MODULE_PATH
        "${IDF_PATH}/tools/cmake"
        "${IDF_PATH}/tools/cmake/third_party"
        ${CMAKE_MODULE_PATH})
include(utilities)
include(git_submodules)                                                   

#--------------------------------------------------------------------------------------
# this part is similar to the platformio.ini section:
#lib_deps=
#    ArduinoJson@5.13.4
#    AsyncTCP
#    ESP Async WebServer
#    ESP8266_SSD1306
#    LiquidCrystal_PCF8574
# ----------------------------------------------------------------------------
# git submodule add https://github.com/bblanchon/ArduinoJson.git; git checkout v5.13.4
# git submodule add https://github.com/me-no-dev/AsyncTCP.git
# git submodule add https://github.com/mathertel/LiquidCrystal_PCF8574
# git submodule add https://github.com/me-no-dev/ESPAsyncWebServer.git
# git submodule add https://github.com/squix78/esp8266-oled-ssd1306.git
# 
# clone the modules, if they are not there yet
git_submodule_check("${CMAKE_CURRENT_SOURCE_DIR}")

#--------------------------------------------------------------------------------------
# include the arduino simulation layer as an extra component:
# git submodule add https://github.com/espressif/arduino-esp32.git; cd arduino-esp32; git submodule update --init --recursive
set(EXTRA_COMPONENT_DIRS 
  ${CMAKE_CURRENT_SOURCE_DIR}/syslib/arduino-esp32
)

#--------------------------------------------------------------------------------------
# now the 'standard' cmake code...
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(ESP32CommandStation)





