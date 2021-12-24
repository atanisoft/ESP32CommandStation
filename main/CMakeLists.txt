###############################################################################
# ESP32 COMMAND STATION
#
# COPYRIGHT (c) 2017-2021 Mike Dunston
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see http://www.gnu.org/licenses
###############################################################################

set(IDF_DEPS
    app_update
    bootloader_support
    driver
    efuse
    esp_adc_cal
    esp_common
    esp_system
    espcoredump
    json
    nvs_flash
    soc
)

# starting in IDF v5 esp_ipc component has been merged into esp_system
if (IDF_VERSION_MAJOR LESS 5)
    list(APPEND IDF_DEPS esp_ipc)
endif()

set(CUSTOM_DEPS
    AccessoryDecoderDB
    Config
    DCC
    HttpServer
    NeoPixelBus
    NvsManager
    OpenMRNIDF
    StatusDisplay
    StatusLED
    TrainDatabase
    TrainSearchProtocol
    ULPADC
    Utils
)

idf_component_register(SRCS ESP32CommandStation.cpp Esp32CoreDumpUtil.cpp WebServer.cpp
                       REQUIRES "${IDF_DEPS} ${CUSTOM_DEPS}")